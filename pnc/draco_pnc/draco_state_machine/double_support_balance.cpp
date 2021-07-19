#include "pnc/draco_pnc/draco_state_machine/double_support_balance.hpp"

DoubleSupportBalance::DoubleSupportBalance(
    const StateIdentifier _state_identifier,
    DracoControlArchitecture *_ctrl_arch, RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  util::PrettyConstructor(2, "DoubleSupportBalance");

  ctrl_arch_ = _ctrl_arch;
  sp_ = DracoStateProvider::getStateProvider();

  b_walking_trigger = false;
  b_swaying_trigger = false;
  b_interpolation_trigger = false;
}

DoubleSupportBalance::~DoubleSupportBalance() {}

void DoubleSupportBalance::firstVisit() {
  std::cout << "draco_states::kBalance" << std::endl;

  ctrl_start_time_ = sp_->curr_time;
  b_walking_trigger = false;
  b_swaying_trigger = false;
  b_interpolation_trigger = false;
}

void DoubleSupportBalance::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // Update Foot Task
  ctrl_arch_->rfoot_tm->UpdateZeroAccCmd();
  ctrl_arch_->lfoot_tm->UpdateZeroAccCmd();
}

void DoubleSupportBalance::lastVisit() {}

bool DoubleSupportBalance::endOfState() {

  if ((b_walking_trigger) && (ctrl_arch_->dcm_tm->footstep_list.size() > 0) &&
      (!(ctrl_arch_->dcm_tm->noRemainingSteps()))) {
    return true;
  }
  if (b_swaying_trigger) {
    return true;
  }
  if (b_interpolation_trigger) {
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

  if (b_swaying_trigger) {
    return draco_states::kSwaying;
  }

  if (b_interpolation_trigger) {
    return draco_states::kBaseInterpolation;
  }
}
