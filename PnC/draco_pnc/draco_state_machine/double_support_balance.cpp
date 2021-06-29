#include "PnC/draco_pnc/draco_state_machine/double_support_balance.hpp"

DoubleSupportBalance::DoubleSupportBalance(
    const StateIdentifier _state_identifier,
    DracoControlArchitecture *_ctrl_arch, RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  myUtils::pretty_constructor(2, "SM: Double Support Balance");

  ctrl_arch_ = _ctrl_arch;
  sp_ = DracoStateProvider::getStateProvider();
}

DoubleSupportBalance::~DoubleSupportBalance() {}

void DoubleSupportBalance::firstVisit() {
  std::cout << "draco_states::kBalance" << std::endl;

  ctrl_start_time_ = sp_->curr_time;
  b_state_switch_trigger = false;
}

void DoubleSupportBalance::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // Update Foot Task
  ctrl_arch_->rfoot_tm->useCurrent();
  ctrl_arch_->lfoot_tm->useCurrent();
}

void DoubleSupportBalance::lastVisit() {}

bool DoubleSupportBalance::endOfState() {

  if ((b_state_switch_trigger) &&
      (ctrl_arch_->dcm_tm->footstep_list.size() > 0) &&
      (!(ctrl_arch_->dcm_tm->noRemainingSteps()))) {
    return true;
  }
  return false;
}

StateIdentifier DoubleSupportBalance::getNextState() {
  int robot_side;
  if (ctrl_arch_->dcm_tm->nextStepRobotSide(robot_side)) {
    if (robot_side == EndEffector::LFoot) {
      return draco_states::kLFootContactTransitionStart;
    } else {
      return draco_states::kRFootContactTransitionStart;
    }
  }
}
