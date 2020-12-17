#include <PnC/DracoPnC/DracoCtrlArchitecture/DracoCtrlArchitecture.hpp>
#include <PnC/DracoPnC/DracoStateMachine/DoubleSupportStand.hpp>

DoubleSupportStand::DoubleSupportStand(
    const StateIdentifier state_identifier_in,
    DracoControlArchitecture* _ctrl_arch, RobotSystem* _robot)
    : StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Double Support Stand");

  // Set Pointer to Control Architecture
  ctrl_arch_ = ((DracoControlArchitecture*)_ctrl_arch);
  taf_container_ = ctrl_arch_->taf_container_;

  // Get State Provider
  sp_ = DracoStateProvider::getStateProvider(robot_);
}

DoubleSupportStand::~DoubleSupportStand() {}

void DoubleSupportStand::firstVisit() {
  std::cout << "[Double Support Stand]" << std::endl;

  ctrl_start_time_ = sp_->curr_time;

  // =========================================================================
  // Initialize CoM Trajectory
  // =========================================================================
  Eigen::VectorXd lfoot_pos =
      robot_->getBodyNodeCoMIsometry(DracoBodyNode::lFootCenter).translation();
  Eigen::VectorXd rfoot_pos =
      robot_->getBodyNodeCoMIsometry(DracoBodyNode::rFootCenter).translation();

  Eigen::VectorXd com_pos = robot_ ->getCoMPosition();

  Eigen::VectorXd target_com_pos = (lfoot_pos + rfoot_pos) / 2.0;
  target_com_pos[2] = target_height_;
  ctrl_arch_->floating_base_lifting_up_manager_
      ->initializeFloatingBaseTrajectory(sp_->curr_time, end_time_,
                                         target_com_pos);

  // =========================================================================
  // Initialize Reaction Force Ramp to Max
  // =========================================================================
  ctrl_arch_->rfoot_front_max_normal_force_manager_->initializeRampToMax(
  0.0, time_to_max_normal_force_);
  ctrl_arch_->rfoot_back_max_normal_force_manager_->initializeRampToMax(
  0.0, time_to_max_normal_force_);
  ctrl_arch_->lfoot_front_max_normal_force_manager_->initializeRampToMax(
  0.0, time_to_max_normal_force_);
  ctrl_arch_->lfoot_back_max_normal_force_manager_->initializeRampToMax(
  0.0, time_to_max_normal_force_);
  // ctrl_arch_->rfoot_max_normal_force_manager_->initializeRampToMax(
  //     0.0, time_to_max_normal_force_);
  // ctrl_arch_->lfoot_max_normal_force_manager_->initializeRampToMax(
  //     0.0, time_to_max_normal_force_);

  // =========================================================================
  // Initialize Task Gain Ramp to Max
  // =========================================================================
  ctrl_arch_->rfoot_pos_hierarchy_manager_->initializeRampToMax(0.0, end_time_);
  ctrl_arch_->rfoot_ori_hierarchy_manager_->initializeRampToMax(0.0, end_time_);
  ctrl_arch_->lfoot_pos_hierarchy_manager_->initializeRampToMax(0.0, end_time_);
  ctrl_arch_->lfoot_ori_hierarchy_manager_->initializeRampToMax(0.0, end_time_);
  ctrl_arch_->com_hierarchy_manager_->initializeRampToMax(0.0, end_time_);
  ctrl_arch_->base_ori_hierarchy_manager_->initializeRampToMax(0.0, end_time_);
}

void DoubleSupportStand::_taskUpdate() {
  // =========================================================================
  // Floating Base
  // =========================================================================
  ctrl_arch_->floating_base_lifting_up_manager_->updateFloatingBaseDesired(
      sp_->curr_time);

  // =========================================================================
  // Foot
  // =========================================================================
  ctrl_arch_->rfoot_trajectory_manager_->useCurrent();
  ctrl_arch_->lfoot_trajectory_manager_->useCurrent();
}

void DoubleSupportStand::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // Compute and update new maximum reaction forces
  ctrl_arch_->lfoot_front_max_normal_force_manager_->updateRampToMaxDesired(
  state_machine_time_);
  ctrl_arch_->lfoot_back_max_normal_force_manager_->updateRampToMaxDesired(
  state_machine_time_);
  ctrl_arch_->rfoot_front_max_normal_force_manager_->updateRampToMaxDesired(
  state_machine_time_);
  ctrl_arch_->rfoot_back_max_normal_force_manager_->updateRampToMaxDesired(
  state_machine_time_);
  // ctrl_arch_->lfoot_max_normal_force_manager_->updateRampToMaxDesired(
  //     state_machine_time_);
  // ctrl_arch_->rfoot_max_normal_force_manager_->updateRampToMaxDesired(
  //     state_machine_time_);

  // Compute and update new hierarchy weights
  ctrl_arch_->rfoot_pos_hierarchy_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->rfoot_ori_hierarchy_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->lfoot_pos_hierarchy_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->lfoot_ori_hierarchy_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->com_hierarchy_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->base_ori_hierarchy_manager_->updateRampToMaxDesired(
      state_machine_time_);
  _taskUpdate();
}

void DoubleSupportStand::lastVisit() {}

bool DoubleSupportStand::endOfState() {
  if (state_machine_time_ > end_time_) {
    return true;
  }
  return false;
}

StateIdentifier DoubleSupportStand::getNextState() {
  return DRACO_STATES::BALANCE;
}

void DoubleSupportStand::initialization(const YAML::Node& node) {
  try {
    myUtils::readParameter(node, "target_pos_duration", end_time_);
    myUtils::readParameter(node, "smoothing_duration", smoothing_dur_);
    myUtils::readParameter(node, "time_to_max_normal_force",
                           time_to_max_normal_force_);
    myUtils::readParameter(node, "target_height", target_height_);

  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }
}
