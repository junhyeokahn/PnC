#include <PnC/DracoPnC/DracoCtrl/DracoWBCController.hpp>
#include <PnC/Filter/Basic/filter.hpp>

DracoWBCController::DracoWBCController(
    DracoTaskAndForceContainer* _taf_container, RobotSystem* _robot) {
  myUtils::pretty_constructor(2, "Draco WBC Controller");
  // Initialize Flag
  b_first_visit_ = true;

  // Initialize Pointer to the Task and Force Container
  taf_container_ = _taf_container;
  robot_ = _robot;

  // Initialize State Provider
  sp_ = DracoStateProvider::getStateProvider(robot_);

  // Initialize WBC
  std::vector<bool> act_list;
  act_list.resize(Draco::n_dof, true);
  for (int i(0); i < Draco::n_vdof; ++i) act_list[i] = false;

  // KinWBC and WBLC
  kin_wbc_ = new KinWBC(act_list);
  wblc_ = new WBLC(act_list);
  awbc_ = new AWBC(act_list);
  wblc_data_ = new WBLC_ExtraData();

  // Initialize torque and qddot des
  tau_cmd_ = Eigen::VectorXd::Zero(Draco::n_adof);
  qddot_cmd_ = Eigen::VectorXd::Zero(Draco::n_adof);

  // Initialize desired pos, vel, acc containers
  des_jpos_ = Eigen::VectorXd::Zero(Draco::n_adof);
  des_jvel_ = Eigen::VectorXd::Zero(Draco::n_adof);
  des_jacc_ = Eigen::VectorXd::Zero(Draco::n_adof);

  q_prev_ = Eigen::VectorXd::Zero(Draco::n_dof);
  dq_prev_ = Eigen::VectorXd::Zero(Draco::n_dof);

  hat_f_ =  Eigen::VectorXd::Zero(6);

  //contact dimension
  dim_contact_ = taf_container_->dim_contact_;

  // robot total mass
  total_mass_ = robot_->getRobotMass();

  x_force_ext_ = new AverageFilter(DracoAux::servo_rate, 0.030, 20.0);
  y_force_ext_ = new AverageFilter(DracoAux::servo_rate, 0.030, 20.0);
  z_force_ext_ = new AverageFilter(DracoAux::servo_rate, 0.030, 100.0);

  x_tau_ext_ = new AverageFilter(DracoAux::servo_rate, 0.030, 50.0);
  y_tau_ext_ = new AverageFilter(DracoAux::servo_rate, 0.030, 50.0);
  z_tau_ext_ = new AverageFilter(DracoAux::servo_rate, 0.030, 50.0);

}

DracoWBCController::~DracoWBCController() {
  delete kin_wbc_;
  delete wblc_;
  delete wblc_data_;

  delete x_force_ext_;
  delete y_force_ext_;
  delete z_force_ext_;
  
  delete x_tau_ext_;
  delete y_tau_ext_;
  delete z_tau_ext_;
}

void DracoWBCController::_PreProcessing_Command() {
  // Update Dynamic Terms
  A_ = robot_->getMassMatrix();
  A_rotor_ = A_;
  for (int i = 0; i < Draco::n_adof; ++i) {
    A_rotor_(i + Draco::n_vdof, i + Draco::n_vdof) += sp_->rotor_inertia[i];
  }
  Ainv_rotor_ = A_rotor_.inverse();
  Ainv_ = robot_->getInvMassMatrix();
  grav_ = robot_->getGravity();
  coriolis_ = robot_->getCoriolis();

  // Clear out local pointers
  task_list_.clear();
  contact_list_.clear();

  // joint pos task update
  Eigen::VectorXd jpos_des = sp_->jpos_ini;
  Eigen::VectorXd jvel_des(Draco::n_adof);
  jvel_des.setZero();
  Eigen::VectorXd jacc_des(Draco::n_adof);
  jacc_des.setZero();
  taf_container_->joint_task_->updateDesired(jpos_des,jvel_des,jacc_des);

  // Grab Variables from the container.
  // Update task and contact list pointers from container object
  // change the tasks for the simulation
  // com, base_ori, rfoot_pos, lfoot_pos, rfoot_ori, lfoot_ori, jpos  
  
  // original
  // for (int i = 0; i < taf_container_->task_list_.size(); i++) {
  //   task_list_.push_back(taf_container_->task_list_[i]);
  // }
  // modified

  Eigen::VectorXi modified_task_idx(3);
  modified_task_idx[0] = 0;
  modified_task_idx[1] = 1;
  modified_task_idx[2] = 6;
  for (int i = 0; i < modified_task_idx.size(); i++) {
    task_list_.push_back(taf_container_->task_list_[modified_task_idx[i]]);
  } 

  for (int i = 0; i < taf_container_->contact_list_.size(); i++) {
    contact_list_.push_back(taf_container_->contact_list_[i]);
  }
  Fd_des_ = taf_container_->Fd_des_;

  // Update Task Jacobians and commands
  for (int i = 0; i < task_list_.size(); i++) {
    task_list_[i]->updateJacobians();
    task_list_[i]->computeCommands();
  }

  // Update Contact Spec
  for (int i = 0; i < contact_list_.size(); i++) {
    contact_list_[i]->updateContactSpec();
  }

  int dim_contact_ptr = 0;
  for (int i = 0; i < contact_list_.size(); i++) {
    int fz_idx = dim_contact_ptr + contact_list_[i]->getFzIndex();
    dim_contact_ptr += contact_list_[i]->getDim();
    wblc_data_->W_rf_[fz_idx] = 0.001;
  }
  // std::cout<<"num contact: "<< contact_list_.size() << std::endl;
  // std::cout<<"W_rf_" << wblc_data_->W_rf_ <<std::endl;
  
  pos_com_ = robot_->getCoMPosition();
  vel_com_ = robot_->getCoMVelocity();
  J_com_ = robot_->getCoMJacobian();
  AM_ = robot_->A_cent_.block(0,0,3,Draco::n_dof);
  H_ = robot_->getDervCentroidMomentum(0.0001);
 
}

void DracoWBCController::getCommand(void* _cmd) {
  // Perform First time visit Initialization
  if (b_first_visit_) {
    firstVisit();
    b_first_visit_ = false;
    awbc_->setGains(kp_, kd_, Kp_, Kd_);
  }

  // Update Dynamic Properties, Task Jacobians, and Contact Jacobians
  _PreProcessing_Command();

  Eigen::VectorXd pos_err;
  Eigen::VectorXd vel_des;
  Eigen::VectorXd acc_des;

  Task* task = task_list_[0]; 
  pos_err = task->pos_err;
  vel_des = task->vel_des;
  acc_des = task->acc_des;

  // std::cout<<"task pos err: " << pos_err << std::endl;
  // std::cout<<"task vel des: " << vel_des << std::endl;
  // std::cout<<"task acc des: " << acc_des << std::endl;

  Eigen::VectorXd des_jpos_full;
  Eigen::VectorXd des_jvel_full;
  Eigen::VectorXd des_jacc_full;

  // solve kinWBC 
  kin_wbc_->FindConfiguration(sp_->q, task_list_, contact_list_, des_jpos_full,
                              des_jvel_full, des_jacc_full);

  des_jpos_ = des_jpos_full.tail(Draco::n_adof);
  des_jvel_ = des_jvel_full.tail(Draco::n_adof);
  des_jacc_ = des_jacc_full.tail(Draco::n_adof);

  // Update settings and qddot_des
  wblc_->updateSetting(A_, Ainv_, coriolis_, grav_);
  Eigen::VectorXd des_jacc_cmd = des_jacc_ + 
        Kp_.cwiseProduct(des_jpos_ - sp_->q.segment(Draco::n_vdof, Draco::n_adof)) +
        Kd_.cwiseProduct(des_jvel_ - sp_->qdot.tail(Draco::n_adof));

  // solve WBLC for obtaining the torque
  wblc_->makeWBLC_Torque(des_jacc_cmd, contact_list_, tau_cmd_, wblc_data_);

  std::vector<Eigen::Vector3d> contact_pos; 
  Eigen::Vector3d temp_pos;
  int num_contact = taf_container_->contact_list_.size();
  Eigen::VectorXd Fr_transformed = Eigen::VectorXd::Zero(6*num_contact);
  Eigen::VectorXd Fr_temp = Eigen::VectorXd::Zero(6);
  Eigen::Isometry3d T1_temp = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_com = Eigen::Isometry3d::Identity(); 
  Eigen::Isometry3d T1_com = Eigen::Isometry3d::Identity();
  Eigen::MatrixXd AdT1_com; 

  T_com.translation() = pos_com_;
  
  if(taf_container_->contact_type_ == 1) // point contact
  {
      contact_pos.push_back(robot_->getBodyNodeIsometry(DracoBodyNode::rFootFront).translation() - pos_com_);
      contact_pos.push_back(robot_->getBodyNodeIsometry(DracoBodyNode::rFootBack).translation() - pos_com_);
      contact_pos.push_back(robot_->getBodyNodeIsometry(DracoBodyNode::lFootFront).translation() - pos_com_);
      contact_pos.push_back(robot_->getBodyNodeIsometry(DracoBodyNode::lFootBack).translation() - pos_com_);
  
      // Fr_temp.segment(3,3) = wblc_data_->Fr_.segment(0,3);
      // T1_temp.translation() = robot_->getBodyNodeIsometry(DracoBodyNode::rFootFront).translation();
      // T1_com = T1_temp.inverse()*T_com;
      // AdT1_com = dart::math::getAdTMatrix(T1_com);
      // Fr_transformed.segment(0,6) = AdT1_com.transpose()*Fr_temp;
      
      // Fr_temp.segment(3,3) = wblc_data_->Fr_.segment(3,3);
      // T1_temp.translation() = robot_->getBodyNodeIsometry(DracoBodyNode::rFootBack).translation();
      // T1_com = T1_temp.inverse()*T_com;
      // AdT1_com = dart::math::getAdTMatrix(T1_com);
      // Fr_transformed.segment(6,6) =  AdT1_com.transpose()*Fr_temp;
      
      // Fr_temp.segment(3,3) = wblc_data_->Fr_.segment(6,3);
      // T1_temp.translation() = robot_->getBodyNodeIsometry(DracoBodyNode::lFootFront).translation();
      // T1_com = T1_temp.inverse()*T_com;
      // AdT1_com = dart::math::getAdTMatrix(T1_com);
      // Fr_transformed.segment(12,3) = AdT1_com.transpose()*Fr_temp;

      // Fr_temp.segment(3,3) = wblc_data_->Fr_.segment(9,3);
      // T1_temp.translation() = robot_->getBodyNodeIsometry(DracoBodyNode::lFootBack).translation();
      // T1_com = T1_temp.inverse()*T_com;
      // AdT1_com = dart::math::getAdTMatrix(T1_com);
      // Fr_transformed.segment(18,3) = AdT1_com.transpose()*Fr_temp;
      
      // whlc contact forces
      Fr_transformed.segment(3,3) = wblc_data_->Fr_.segment(0,3);
      Fr_transformed.segment(9,3) = wblc_data_->Fr_.segment(3,3);
      Fr_transformed.segment(15,3) = wblc_data_->Fr_.segment(6,3);
      Fr_transformed.segment(21,3) = wblc_data_->Fr_.segment(9,3);

      // measured contact forces
      // Fr_transformed.segment(3,3) = sp_->r_front_rf.segment(3,3);
      // Fr_transformed.segment(9,3) = sp_->r_back_rf.segment(3,3);
      // Fr_transformed.segment(15,3) = sp_->l_front_rf.segment(3,3);
      // Fr_transformed.segment(21,3) = sp_->l_back_rf.segment(3,3);

      // myUtils::pretty_print(wblc_data_->Fr_, std::cout, "wblc_data_->Fr_");
      // myUtils::pretty_print(Fr_temp, std::cout, "Fr_temp");
      // myUtils::pretty_print(Fr_transformed, std::cout, "Fr_transformed");
  }

  else if (taf_container_->contact_type_ == 2)  //surface contact
  {
      contact_pos.push_back(robot_->getBodyNodeIsometry(DracoBodyNode::rFootCenter).translation() - pos_com_);
      contact_pos.push_back(robot_->getBodyNodeIsometry(DracoBodyNode::lFootCenter).translation() - pos_com_);
  } 

  Eigen::Vector3d ext_pos = robot_->getBodyNodeIsometry(DracoBodyNode::Torso).translation() - pos_com_;
  Eigen::MatrixXd ext_Jacobian = robot_->getBodyNodeJacobian(DracoBodyNode::Torso);

  // Cordinate transform 
  
  // setting for adaptation of WBC
  awbc_->updateJointSetting(sp_->q, q_prev_, sp_->qdot, dq_prev_, des_jpos_full , des_jvel_full);
  awbc_->updateMassSetting(A_, A_prev_, total_mass_);
  awbc_->updateContactSetting(Fr_transformed, contact_pos, ext_pos, ext_Jacobian);
  awbc_->updateCDSetting(pos_com_, pos_com_prev_, J_com_, J_com_prev_, AM_, AM_prev_, H_, H_prev_);

  // myUtils::pretty_print(H_, std::cout, "H");

  // Estimate External force 
  // awbc_->EstimateExtforce();

  Eigen::VectorXd Kp_a;
  Eigen::VectorXd Kd_a;

  awbc_->AdaptGains( contact_list_, Kp_a, Kd_a);

  // myUtils::pretty_print(Kp_, std::cout, "Kp_");
  // myUtils::pretty_print(Kd_, std::cout, "Kd_");
  // myUtils::pretty_print(Kp_a, std::cout, "Kp_a");
  // myUtils::pretty_print(Kd_a, std::cout, "Kd_a");

  hat_f_.segment(3,3) = awbc_->hat_f_t_;
  hat_f_.segment(0,3) = awbc_->hat_tau_t_;

  x_tau_ext_->input(hat_f_[0]);
  hat_f_[0] = x_tau_ext_->output();
  y_tau_ext_->input(hat_f_[1]);
  hat_f_[1] = y_tau_ext_->output();
  z_tau_ext_->input(hat_f_[2]);
  hat_f_[2] = z_tau_ext_->output();

  x_force_ext_->input(hat_f_[3]);
  hat_f_[3] = x_force_ext_->output();
  y_force_ext_->input(hat_f_[4]);
  hat_f_[4] = y_force_ext_->output();
  z_force_ext_->input(hat_f_[5]);
  hat_f_[5] = z_force_ext_->output();

  // myUtils::pretty_print(wblc_data_->Fr_, std::cout, "ground reaction force");
  // myUtils::pretty_print(hat_f, std::cout, "ext_force");

  Eigen::MatrixXd ext_Jacobian_inv;
  myUtils::weightedInverse(ext_Jacobian, Ainv_, ext_Jacobian_inv);

  Eigen::VectorXd F_cmd = ext_Jacobian_inv.transpose()*tau_cmd_;

  // Set Command
  for (int i(0); i < Draco::n_adof; ++i) {
    ((DracoCommand*)_cmd)->jtrq[i] = tau_cmd_[i];
    ((DracoCommand*)_cmd)->q[i] = des_jpos_[i];
    ((DracoCommand*)_cmd)->qdot[i] = des_jvel_[i];
  }

  ((DracoCommand*)_cmd)->Fr_estimated = Fr_transformed;
  ((DracoCommand*)_cmd)->Fr_ext = hat_f_;
  // ((DracoCommand*)_cmd)->Fr_ext = hat_f - F_cmd;
  
  // update previous joint configuration and velocity
  q_prev_ = sp_->q;
  dq_prev_ = sp_->qdot;
  A_prev_ = A_;
  pos_com_prev_ = pos_com_;
  vel_com_prev_ = vel_com_;
  J_com_prev_ = J_com_;
  AM_prev_ = AM_;
  H_prev_ = H_;
  // myUtils::pretty_print(((DracoCommand*)_cmd)->jtrq, std::cout, "jtrq");
  // myUtils::pretty_print(((DracoCommand*)_cmd)->q, std::cout, "q");
  // myUtils::pretty_print(((DracoCommand*)_cmd)->qdot, std::cout, "qdot");
}


void DracoWBCController::firstVisit() {
  q_prev_ = sp_->q;
  dq_prev_ = sp_->qdot;
  A_prev_ = robot_->getMassMatrix();
  pos_com_prev_ = robot_->getCoMPosition();
  vel_com_prev_ = robot_->getCoMVelocity();
  J_com_prev_ = robot_->getCoMJacobian();
  AM_prev_ = robot_->A_cent_.block(0,0,3,Draco::n_dof);
  H_prev_ = robot_->getDervCentroidMomentum(0.0001);

  ((AverageFilter*)x_tau_ext_)->initialization(hat_f_[0]);
  ((AverageFilter*)y_tau_ext_)->initialization(hat_f_[1]);
  ((AverageFilter*)z_tau_ext_)->initialization(hat_f_[2]);

  ((AverageFilter*)x_force_ext_)->initialization(hat_f_[3]);
  ((AverageFilter*)y_force_ext_)->initialization(hat_f_[4]);
  ((AverageFilter*)z_force_ext_)->initialization(hat_f_[5]);

 }

void DracoWBCController::ctrlInitialization(const YAML::Node& node) {
  // WBC Defaults
  lambda_xddot_ = 1000.0;
  lambda_qddot_ = 100.0;            // Generalized Coord Acceleration
  lambda_rf_ = 0.1;                 // Reaction Force Regularization
  b_enable_torque_limits_ = true;   // Enable WBC torque limits

  Kp_ = Eigen::VectorXd::Constant(Draco::n_dof, 300);
  Kd_ = Eigen::VectorXd::Constant(Draco::n_dof, 15);

  // Load Custom Parmams ----------------------------------
  try {
    // Load WBC Parameters
    myUtils::readParameter(node, "lambda_xddot", lambda_xddot_);
    myUtils::readParameter(node, "lambda_qddot", lambda_qddot_);
    myUtils::readParameter(node, "lambda_rf", lambda_rf_);
    myUtils::readParameter(node, "enable_torque_limits",
                           b_enable_torque_limits_);
    myUtils::readParameter(node, "Kp", Kp_);
    myUtils::readParameter(node, "Kd", Kd_);

  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }

  // ----------------------------------
  // Setting default weighting for optimization problem
  wblc_data_->W_qddot_ = Eigen::VectorXd::Constant(Draco::n_dof, lambda_qddot_);
  wblc_data_->W_rf_ = Eigen::VectorXd::Constant(dim_contact_, lambda_rf_);
  wblc_data_->W_xddot_ = Eigen::VectorXd::Constant(dim_contact_, lambda_xddot_);

  // torque limit default setting
  wblc_data_->tau_min_ = Eigen::VectorXd::Constant(Draco::n_adof, -2500.);
  wblc_data_->tau_max_ = Eigen::VectorXd::Constant(Draco::n_adof, 2500.);
 
  try {
    YAML::Node simulation_cfg =
        YAML::LoadFile(THIS_COM "Config/Draco/SIMULATION.yaml");
    YAML::Node control_cfg = simulation_cfg["control_configuration"];
    myUtils::readParameter(control_cfg, "kp", kp_);
    myUtils::readParameter(control_cfg, "kd", kd_);
  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
  }

}
