#include <PnC/A1PnC/A1Ctrl/A1MainController.hpp>

A1MainController::A1MainController(
  A1TaskAndForceContainer* _taf_container, RobotSystem* _robot) {
  myUtils::pretty_constructor(2, "A1 Main Controller");

  // Initialize Pointer to the Task and Force Container
  taf_container_ = _taf_container;
  robot_ = _robot;

  // Initialize State Provider
  sp_ = A1StateProvider::getStateProvider(robot_);

  // Initialize WBIC
  std::vector<bool> act_list;
  act_list.resize(A1::n_dof, true);
  for (int i(0); i < A1::n_vdof; ++i) act_list[i] = false;
  wbic_ = new WBIC(act_list, task_list_, contact_list_);
  kin_wbc_ = new KinWBC(act_list);
  wbic_data_ = new WBIC_ExtraData();


  tau_cmd_ = Eigen::VectorXd::Zero(A1::n_adof);
  qddot_cmd_ = Eigen::VectorXd::Zero(A1::n_adof);

  wbic_data_->_W_floating = Eigen::VectorXd::Constant(6, 1000.);
  wbic_data_->_W_rf = Eigen::VectorXd::Constant(12, 1.);

  _Kp_joint.resize(3, 5.); // num_leg_joint, value
  _Kd_joint.resize(3, 1.5);

  // Initialize desired pos, vel, acc containers
  des_jpos_ = Eigen::VectorXd::Zero(A1::n_adof);
  des_jvel_ = Eigen::VectorXd::Zero(A1::n_adof);
  des_jacc_ = Eigen::VectorXd::Zero(A1::n_adof);
}

A1MainController::~A1MainController() {
  delete wbic_;
  delete kin_wbc_;
  delete wbic_data_;
}

void A1MainController::_PreProcessing_Command() {
  // Update Dynamic Terms
  A_ = robot_->getMassMatrix();
  // myUtils::pretty_print(A_, std::cout, "A");
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
  // Fd_des_ = taf_container_->Fd_des_;

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

Eigen::VectorXd A1MainController::getCommand(void* _cmd, bool change_weights) {
  if(change_weights) {
    wbic_data_->_W_floating = Eigen::VectorXd::Constant(6, 0.1);
    wbic_data_->_W_rf = Eigen::VectorXd::Constant(12, 1.);
  }
  // Update Dynamic Terms, Task Jacobians, and Contact Jacobians
  _PreProcessing_Command();

  // Update Task Hierarchy
  Eigen::VectorXd w_task_hierarchy_ = Eigen::VectorXd::Zero(task_list_.size());
  for (int i = 0; i < task_list_.size(); i++) {
    w_task_hierarchy_[i] = task_list_[i]->getHierarchyWeight();
  }

  // IK Module
  kin_wbc_->FindConfiguration(robot_->getQ(), task_list_, contact_list_,
                              des_jpos_, des_jvel_);
  // myUtils::pretty_print(des_jpos_, std::cout, "des_jpos_ (main_controller)");
  // myUtils::pretty_print(des_jvel_, std::cout, "des_jvel_ (main_controller)");
  // WBIC
  wbic_->updateSetting(A_, Ainv_, coriolis_, grav_);
  Eigen::VectorXd Fr_result_;
  wbic_->makeTorque(contact_list_, task_list_, tau_cmd_, Fr_result_, wbic_data_);
  // if(Fr_result_.size() < 12){
  //     std::cout << "Fr_result_ size wrong" << std::endl;
  // } else {
  //     sp_->final_reaction_forces = Fr_result_;
  // }
  // myUtils::pretty_print(tau_cmd_, std::cout, "tau_cmd_ [Main Controller]");
  // myUtils::pretty_print(Fr_result_, std::cout, "Fr_res");

  // std::cout << "Robot Mass = " << robot_->getRobotMass() << std::endl;

  // Set Command
  for (int i(0); i < A1::n_adof; ++i) {
    ((A1Command*)_cmd)->jtrq[i] = tau_cmd_[i];
    ((A1Command*)_cmd)->q[i] = des_jpos_[i];
    ((A1Command*)_cmd)->qdot[i] = des_jvel_[i];
  }

  return Fr_result_;
}

void A1MainController::ctrlInitialization(const YAML::Node& node) {
  Eigen::VectorXd tau_min = robot_->GetTorqueLowerLimits().segment(
      A1::n_vdof, A1::n_adof);
  Eigen::VectorXd tau_max = robot_->GetTorqueUpperLimits().segment(
      A1::n_vdof, A1::n_adof);
  // wbc_->setTorqueLimits(tau_min, tau_max);

}
