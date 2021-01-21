#include <PnC/LaikagoPnC/LaikagoCtrlArchitecture/LaikagoCtrlArchitecture.hpp>
#include <PnC/LaikagoPnC/LaikagoStateMachine/DoubleSupportBalance.hpp>

DoubleSupportBalance::DoubleSupportBalance(
    const StateIdentifier state_identifier_in,
    LaikagoControlArchitecture* _ctrl_arch, RobotSystem* _robot)
    : StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Double Support Balance");

  // Set Trigger to false
  state_switch_button_trigger_ = false;

  // Set Pointer to Control Architecture
  ctrl_arch_ = ((LaikagoControlArchitecture*)_ctrl_arch);
  taf_container_ = ctrl_arch_->taf_container_;
  // Get State Provider
  sp_ = LaikagoStateProvider::getStateProvider(robot_);
}

DoubleSupportBalance::~DoubleSupportBalance() {}

void DoubleSupportBalance::firstVisit() {
  std::cout << "[Double Support Balance] Start" << std::endl;
  // Reset Flags
  state_switch_button_trigger_ = false;
  ctrl_start_time_ = sp_->curr_time;
}

void DoubleSupportBalance::_taskUpdate() {
  // =========================================================================
  // Foot, Floating Base
  // =========================================================================
  ctrl_arch_->rfoot_trajectory_manager_->useCurrent();
  ctrl_arch_->lfoot_trajectory_manager_->useCurrent();
}

void DoubleSupportBalance::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  _taskUpdate();
}

void DoubleSupportBalance::lastVisit() {}

bool DoubleSupportBalance::endOfState() {
  // Also check if footstep list is non-zero
  if (state_switch_button_trigger_ &&
      (ctrl_arch_->dcm_trajectory_manager_->footstep_list_.size() > 0) &&
      (!ctrl_arch_->dcm_trajectory_manager_->noRemainingSteps())) {
    return true;
  }
  return false;
}

StateIdentifier DoubleSupportBalance::getNextState() {
  int robot_side;
  // Check if there's a valid step
  if (ctrl_arch_->dcm_trajectory_manager_->nextStepRobotSide(robot_side)) {
    // Check which side is the next footstep
    if (robot_side == LEFT_ROBOT_SIDE) {
      return Laikago_STATES::LL_CONTACT_TRANSITION_START;
    } else {
      return Laikago_STATES::RL_CONTACT_TRANSITION_START;
    }
  }
}

void DoubleSupportBalance::initialization(const YAML::Node& node) {}
