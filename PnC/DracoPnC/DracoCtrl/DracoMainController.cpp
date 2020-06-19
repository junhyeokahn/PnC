#include <PnC/DracoPnC/DracoCtrl/DracoMainController.hpp>

DracoMainController::DracoMainController(
    DracoTaskAndForceContainer* _taf_container, RobotSystem* _robot) {
  myUtils::pretty_constructor(2, "Draco Main Controller");
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
  wbc_ = new WBC(act_list);

  tau_cmd_ = Eigen::VectorXd::Zero(Draco::n_adof);
  qddot_cmd_ = Eigen::VectorXd::Zero(Draco::n_adof);

  // Initialize Joint Integrator
  wbc_dt_ = DracoAux::servo_rate;
  joint_integrator_ = new JointIntegrator(Draco::n_adof, wbc_dt_);

  // Initialize desired pos, vel, acc containers
  des_jpos_ = Eigen::VectorXd::Zero(Draco::n_adof);
  des_jvel_ = Eigen::VectorXd::Zero(Draco::n_adof);
  des_jacc_ = Eigen::VectorXd::Zero(Draco::n_adof);
}

DracoMainController::~DracoMainController() {
  delete wbc_;
  delete joint_integrator_;
}

void DracoMainController::_PreProcessing_Command() {
  // Update Dynamic Terms
  A_ = robot_->getMassMatrix();
  Ainv_ = robot_->getInvMassMatrix();
  grav_ = robot_->getGravity();
  coriolis_ = robot_->getCoriolis();

  // Clear out local pointers
  task_list_.clear();
  contact_list_.clear();

  // Grab Variables from the container.
  // Update task and contact list pointers from container object
  for (int i = 0; i < taf_container_->task_list_.size(); i++) {
    task_list_.push_back(taf_container_->task_list_[i]);
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
}

void DracoMainController::getCommand(void* _cmd) {
  // Perform First time visit Initialization
  if (b_first_visit_) {
    firstVisit();
    b_first_visit_ = false;
  }

  // Update Dynamic Terms, Task Jacobians, and Contact Jacobians
  _PreProcessing_Command();

  // Update Task Hierarchy
  Eigen::VectorXd w_task_hierarchy_ = Eigen::VectorXd::Zero(task_list_.size());
  for (int i = 0; i < task_list_.size(); i++) {
    w_task_hierarchy_[i] = task_list_[i]->getHierarchyWeight();
  }

  // Set QP weights
  double local_w_contact_weight =
      w_contact_weight_ / (robot_->getRobotMass() * 9.81);
  wbc_->setQPWeights(w_task_hierarchy_, local_w_contact_weight);
  wbc_->setRegularizationTerms(lambda_qddot_, lambda_Fr_);

  // QP dec variable results
  Eigen::VectorXd qddot_res;
  Eigen::VectorXd Fr_res;

  // Update QP and solve
  wbc_->updateSetting(A_, Ainv_, coriolis_, grav_);
  wbc_->solve(task_list_, contact_list_, Fd_des_, tau_cmd_, qddot_cmd_);

  // Get Results
  wbc_->getQddotResult(qddot_res);
  sp_->qddot_cmd = qddot_res;
  wbc_->getFrResult(Fr_res);
  for (int i = 0; i < 3; ++i) {
    sp_->r_rf_front_des[i] = Fr_res[i];
    sp_->r_rf_back_des[i] = Fr_res[i + 3];
    sp_->l_rf_front_des[i] = Fr_res[i + 6];
    sp_->l_rf_back_des[i] = Fr_res[i + 9];
  }

  // Integrate Joint Velocities and Positions
  des_jacc_ = qddot_cmd_;
  if (joint_integrator_->isInitialized()) {
    joint_integrator_->integrate(
        des_jacc_, sp_->qdot.segment(Draco::n_vdof, Draco::n_adof),
        sp_->q.segment(Draco::n_vdof, Draco::n_adof), des_jvel_, des_jpos_);
  } else {
    des_jpos_ = sp_->q.segment(Draco::n_vdof, Draco::n_adof);
    des_jvel_ = sp_->qdot.segment(Draco::n_vdof, Draco::n_adof);
  }

  // myUtils::pretty_print(tau_cmd_, std::cout, "tau_cmd_");
  // myUtils::pretty_print(Fr_res, std::cout, "Fr_res");

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

void DracoMainController::initializeJointIntegrator() {
  Eigen::VectorXd jpos_ini = sp_->q.segment(Draco::n_vdof, Draco::n_adof);
  joint_integrator_->initializeStates(Eigen::VectorXd::Zero(Draco::n_adof),
                                      jpos_ini);
}

void DracoMainController::firstVisit() {}

void DracoMainController::ctrlInitialization(const YAML::Node& node) {
  // WBC Defaults
  wbc_dt_ = DracoAux::servo_rate;
  w_contact_weight_ = 1e-3;        // Contact Weight
  lambda_qddot_ = 1e-8;            // Generalized Coord Acceleration
  lambda_Fr_ = 1e-8;               // Reaction Force Regularization
  b_enable_torque_limits_ = true;  // Enable WBC torque limits

  // Joint Integrator Defaults
  vel_freq_cutoff_ = 2.0;  // Hz
  pos_freq_cutoff_ = 1.0;  // Hz
  max_pos_error_ = 0.2;    // Radians

  // Load Custom Parmams ----------------------------------
  try {
    // Load WBC Parameters
    myUtils::readParameter(node, "w_contact_weight", w_contact_weight_);
    myUtils::readParameter(node, "lambda_qddot", lambda_qddot_);
    myUtils::readParameter(node, "lambda_Fr", lambda_Fr_);
    myUtils::readParameter(node, "enable_torque_limits",
                           b_enable_torque_limits_);

    // Load Integration Parameters
    myUtils::readParameter(node, "velocity_freq_cutoff", vel_freq_cutoff_);
    myUtils::readParameter(node, "position_freq_cutoff", pos_freq_cutoff_);
    myUtils::readParameter(node, "max_position_error", max_pos_error_);
  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }
  // ----------------------------------

  // Set WBC Parameters
  // Enable Torque Limits
  wbc_->enableTorqueLimits(b_enable_torque_limits_);
  Eigen::VectorXd tau_min =
      robot_->GetTorqueLowerLimits().segment(Draco::n_vdof, Draco::n_adof);
  Eigen::VectorXd tau_max =
      robot_->GetTorqueUpperLimits().segment(Draco::n_vdof, Draco::n_adof);
  wbc_->setTorqueLimits(tau_min, tau_max);

  // Set Joint Integrator Parameters
  // Use cutoff = 0.0 to perform traditional integration
  joint_integrator_->setVelocityFrequencyCutOff(vel_freq_cutoff_);
  joint_integrator_->setPositionFrequencyCutOff(pos_freq_cutoff_);
  // Set Maximum Current Position Deviation
  joint_integrator_->setMaxPositionError(max_pos_error_);
  // Set Joint velocity and position hardware limits
  joint_integrator_->setVelocityBounds(
      robot_->getVelocityLowerLimits().segment(Draco::n_vdof, Draco::n_adof),
      robot_->getVelocityUpperLimits().segment(Draco::n_vdof, Draco::n_adof));
  joint_integrator_->setPositionBounds(
      robot_->getPositionLowerLimits().segment(Draco::n_vdof, Draco::n_adof),
      robot_->getPositionUpperLimits().segment(Draco::n_vdof, Draco::n_adof));
}
