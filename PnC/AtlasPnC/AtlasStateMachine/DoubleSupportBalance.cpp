#include <PnC/AtlasPnC/AtlasStateMachine/DoubleSupportBalance.hpp>

DoubleSupportBalance::DoubleSupportBalance(
    const StateIdentifier _state_identifier,
    AtlasControlArchitecture *_ctrl_arch, RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  myUtils::pretty_constructor(2, "SM: Double Support Balance");

  atlas_ctrl_arch_ = _ctrl_arch;
  sp_ = AtlasStateProvider::getStateProvider(_robot);
}

DoubleSupportBalance::~DoubleSupportBalance() {}

void DoubleSupportBalance::firstVisit() {
  std::cout << "AtlasState::Balance" << std::endl;

  ctrl_start_time_ = sp_->curr_time;
  b_state_switch_trigger = false;
}

void DoubleSupportBalance::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // Update Foot Task
  atlas_ctrl_arch_->rfoot_tm->useCurrent();
  atlas_ctrl_arch_->lfoot_tm->useCurrent();
}

void DoubleSupportBalance::lastVisit() {}

bool DoubleSupportBalance::endOfState() {

  if ((b_state_switch_trigger) &&
      (atlas_ctrl_arch_->dcm_tm->footstep_list.size() > 0) &&
      (!(atlas_ctrl_arch_->dcm_tm->noRemainingSteps()))) {
    return true;
  }
  return false;
}

StateIdentifier DoubleSupportBalance::getNextState() {
  int robot_side;
  if (atlas_ctrl_arch_->dcm_tm->nextStepRobotSide(robot_side)) {
    if (robot_side == EndEffector::LFoot) {
      return AtlasStates::LFootContactTransitionStart;
    } else {
      return AtlasStates::RFootContactTransitionStart;
    }
  }
}