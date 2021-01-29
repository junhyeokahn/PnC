#include <PnC/A1PnC/A1CtrlArchitecture/A1CtrlArchitecture.hpp>
#include <PnC/A1PnC/A1StateMachine/QuadSupportStand.hpp>

QuadSupportStand::QuadSupportStand(
    const StateIdentifier state_identifier_in,
    A1ControlArchitecture* _ctrl_arch, RobotSystem* _robot)
    : StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Quad Support Stand");

  // Set Pointer to Control Architecture
  ctrl_arch_ = ((A1ControlArchitecture*)_ctrl_arch);
  taf_container_ = ctrl_arch_->taf_container_;

  // Get State Provider
  sp_ = A1StateProvider::getStateProvider(robot_);
}

QuadSupportStand::~QuadSupportStand() {}

void QuadSupportStand::firstVisit() {
  std::cout << "[Quad Support Stand]" << std::endl;

  ctrl_start_time_ = sp_->curr_time;

  // =========================================================================
  // Initialize CoM Trajectory
  // =========================================================================
  Eigen::VectorXd flfoot_pos =
      robot_->getBodyNodeCoMIsometry(A1BodyNode::FL_foot).translation();
  Eigen::VectorXd frfoot_pos =
      robot_->getBodyNodeCoMIsometry(A1BodyNode::FR_foot).translation();
  Eigen::VectorXd rlfoot_pos =
      robot_->getBodyNodeCoMIsometry(A1BodyNode::RL_foot).translation();
  Eigen::VectorXd rrfoot_pos =
      robot_->getBodyNodeCoMIsometry(A1BodyNode::RR_foot).translation();
  Eigen::VectorXd target_com_pos = (flfoot_pos + frfoot_pos + rlfoot_pos + rrfoot_pos) / 4.0;
  target_com_pos[2] = target_height_;
  ctrl_arch_->floating_base_lifting_up_manager_
      ->initializeFloatingBaseTrajectory(sp_->curr_time, end_time_,
                                         target_com_pos);

  // =========================================================================
  // Initialize Reaction Force Ramp to Max
  // =========================================================================
  ctrl_arch_->frfoot_max_normal_force_manager_->initializeRampToMax(
      0.0, time_to_max_normal_force_);
  ctrl_arch_->flfoot_max_normal_force_manager_->initializeRampToMax(
      0.0, time_to_max_normal_force_);
  ctrl_arch_->rrfoot_max_normal_force_manager_->initializeRampToMax(
      0.0, time_to_max_normal_force_);
  ctrl_arch_->rlfoot_max_normal_force_manager_->initializeRampToMax(
      0.0, time_to_max_normal_force_);

  // =========================================================================
  // Initialize Task Gain Ramp to Max
  // =========================================================================
  ctrl_arch_->frfoot_pos_hierarchy_manager_->initializeRampToMax(0.0, end_time_);
  ctrl_arch_->rrfoot_pos_hierarchy_manager_->initializeRampToMax(0.0, end_time_);
  ctrl_arch_->flfoot_pos_hierarchy_manager_->initializeRampToMax(0.0, end_time_);
  ctrl_arch_->rlfoot_pos_hierarchy_manager_->initializeRampToMax(0.0, end_time_);
  ctrl_arch_->com_hierarchy_manager_->initializeRampToMax(0.0, end_time_);
  ctrl_arch_->base_ori_hierarchy_manager_->initializeRampToMax(0.0, end_time_);
}

void QuadSupportStand::_taskUpdate() {
  // =========================================================================
  // Floating Base
  // =========================================================================
  ctrl_arch_->floating_base_lifting_up_manager_->updateFloatingBaseDesired(
      sp_->curr_time);
  // =========================================================================
  // Foot
  // =========================================================================
  // ctrl_arch_->rfoot_trajectory_manager_->useCurrent();
  // ctrl_arch_->lfoot_trajectory_manager_->useCurrent();
}

void QuadSupportStand::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  ctrl_arch_->flfoot_max_normal_force_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->frfoot_max_normal_force_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->rlfoot_max_normal_force_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->rrfoot_max_normal_force_manager_->updateRampToMaxDesired(
      state_machine_time_);

  // Compute and update new hierarchy weights
  ctrl_arch_->frfoot_pos_hierarchy_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->rrfoot_pos_hierarchy_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->flfoot_pos_hierarchy_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->rlfoot_pos_hierarchy_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->com_hierarchy_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->base_ori_hierarchy_manager_->updateRampToMaxDesired(
      state_machine_time_);
  _taskUpdate();
}

void QuadSupportStand::lastVisit() {std::cout << "[lastVisit Quad Support Stand]" << std::endl;}

bool QuadSupportStand::endOfState() {
  if (state_machine_time_ > end_time_) {
    return true;
  }
  return false;
}

StateIdentifier QuadSupportStand::getNextState() {
  return A1_STATES::BALANCE;
}

void QuadSupportStand::initialization(const YAML::Node& node) {
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
  std::cout << "QuadSupportStand initialization" << std::endl;
}
