#include <PnC/DracoPnC/DracoCtrl/DracoWBCController.hpp>

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
  wblc_data_ = new WBLC_ExtraData();

  // Initialize torque and qddot des
  tau_cmd_ = Eigen::VectorXd::Zero(Draco::n_adof);
  qddot_cmd_ = Eigen::VectorXd::Zero(Draco::n_adof);

  // Initialize desired pos, vel, acc containers
  des_jpos_ = Eigen::VectorXd::Zero(Draco::n_adof);
  des_jvel_ = Eigen::VectorXd::Zero(Draco::n_adof);
  des_jacc_ = Eigen::VectorXd::Zero(Draco::n_adof);

  // contact dimension
  dim_contact_ = taf_container_->dim_contact_;
}

DracoWBCController::~DracoWBCController() {
  delete kin_wbc_;
  delete wblc_;
  delete wblc_data_;
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

  // Grab Variables from the container.
  // Update task and contact list pointers from container object
  // change the tasks for the simulation
  // com, base_ori, rfoot_pos, lfoot_pos, rfoot_ori, lfoot_ori, jpos

  // original
  // for (int i = 0; i < taf_container_->task_list_.size(); i++) {
  //   task_list_.push_back(taf_container_->task_list_[i]);
  // }
  // modified
  Eigen::VectorXi modified_task_idx(2);
  modified_task_idx[0] = 8;
  modified_task_idx[1] = 7;
  taf_container_->task_list_[8]->updateDesired(Eigen::VectorXd::Zero(2),
                                               Eigen::VectorXd::Zero(2),
                                               Eigen::VectorXd::Zero(2));
  // modified_task_idx[2] = 6;
  for (int i = 0; i < modified_task_idx.size(); i++) {
    task_list_.push_back(taf_container_->task_list_[modified_task_idx[i]]);
  }

  for (int i = 0; i < taf_container_->contact_list_.size(); i++) {
    contact_list_.push_back(taf_container_->contact_list_[i]);
  }
  Fd_des_ = taf_container_->Fd_des_;

  // Update Task Jacobians and commands
  for (int i = 0; i < modified_task_idx.size(); i++) {
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
    wblc_data_->W_rf_[fz_idx] = 0.01;
  }
}

void DracoWBCController::getCommand(void* _cmd) {
  // Perform First time visit Initialization
  if (b_first_visit_) {
    firstVisit();
    b_first_visit_ = false;
  }

  // Update Dynamic Properties, Task Jacobians, and Contact Jacobians
  _PreProcessing_Command();

  // solve kinWBC
  kin_wbc_->FindConfiguration(sp_->q, task_list_, contact_list_, des_jpos_,
                              des_jvel_, des_jacc_);

  // Update settings and qddot_des
  wblc_->updateSetting(A_, Ainv_, coriolis_, grav_);
  Eigen::VectorXd des_jacc_cmd =
      des_jacc_ +
      Kp_.cwiseProduct(des_jpos_ -
                       sp_->q.segment(Draco::n_vdof, Draco::n_adof)) +
      Kd_.cwiseProduct(des_jvel_ - sp_->qdot.tail(Draco::n_adof));

  // solve WBLC for obtaining the torque
  myUtils::pretty_print(des_jacc_cmd, std::cout, "des_jacc");
  wblc_->makeWBLC_Torque(des_jacc_cmd, contact_list_, tau_cmd_, wblc_data_);

  // myUtils::pretty_print(gamma, std::cout, "gamma");

  // Set Command
  for (int i(0); i < Draco::n_adof; ++i) {
    ((DracoCommand*)_cmd)->jtrq[i] = tau_cmd_[i];
    ((DracoCommand*)_cmd)->q[i] = des_jpos_[i];
    ((DracoCommand*)_cmd)->qdot[i] = des_jvel_[i];
  }

  // myUtils::pretty_print(((DracoCommand*)_cmd)->jtrq, std::cout, "jtrq");
  // myUtils::pretty_print(((DracoCommand*)_cmd)->q, std::cout, "q");
  // myUtils::pretty_print(((DracoCommand*)_cmd)->qdot, std::cout, "qdot");
}

void DracoWBCController::firstVisit() {}

void DracoWBCController::ctrlInitialization(const YAML::Node& node) {
  // WBC Defaults
  lambda_xddot_ = 1000.0;
  lambda_qddot_ = 100.0;           // Generalized Coord Acceleration
  lambda_rf_ = 0.1;                // Reaction Force Regularization
  b_enable_torque_limits_ = true;  // Enable WBC torque limits

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
}
