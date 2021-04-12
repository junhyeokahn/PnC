#include <PnC/A1PnC/A1CtrlArchitecture/A1CtrlArchitecture.hpp>
#include <PnC/A1PnC/A1StateMachine/QuadSupportBalance.hpp>

QuadSupportBalance::QuadSupportBalance(
    const StateIdentifier state_identifier_in,
    A1ControlArchitecture* _ctrl_arch, RobotSystem* _robot)
    : StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Quad Support Balance");

  // Set Trigger to false
  state_switch_button_trigger_ = false;

  // Set Pointer to Control Architecture
  ctrl_arch_ = ((A1ControlArchitecture*)_ctrl_arch);
  taf_container_ = ctrl_arch_->taf_container_;
  // Get State Provider
  sp_ = A1StateProvider::getStateProvider(robot_);
}

QuadSupportBalance::~QuadSupportBalance() {}

void QuadSupportBalance::firstVisit() {
  std::cout << "[Quad Support Balance] Start" << std::endl;
  // Reset Flags
  state_switch_button_trigger_ = false;
  ctrl_start_time_ = sp_->curr_time;
}

void QuadSupportBalance::_taskUpdate() {

}

void QuadSupportBalance::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  _taskUpdate();
}

void QuadSupportBalance::lastVisit() {}

bool QuadSupportBalance::endOfState() {
  // Also check if footstep list is non-zero
  if (state_switch_button_trigger_) {  //  &&
    //     (ctrl_arch_->dcm_trajectory_manager_->footstep_list_.size() > 0) &&
    //     (!ctrl_arch_->dcm_trajectory_manager_->noRemainingSteps())) {
    return true;
  }
  return false;
}

StateIdentifier QuadSupportBalance::getNextState() {
  return A1_STATES::FL_CONTACT_TRANSITION_START;
  // int robot_side;
  // Check if there's a valid step
  // if (ctrl_arch_->dcm_trajectory_manager_->nextStepRobotSide(robot_side)) {
  //   // Check which side is the next footstep
  //   if (robot_side == LEFT_ROBOT_SIDE) {
  //     return DRACO_STATES::LL_CONTACT_TRANSITION_START;
  //   } else {
  //     return DRACO_STATES::RL_CONTACT_TRANSITION_START;
  //   }
  // }
}

void QuadSupportBalance::initialization(const YAML::Node& node) {}
