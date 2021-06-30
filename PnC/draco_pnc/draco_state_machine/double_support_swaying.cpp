#include "PnC/draco_pnc/draco_state_machine/double_support_swaying.hpp"

DoubleSupportSwaying::DoubleSupportSwaying(
    const StateIdentifier _state_identifier,
    DracoControlArchitecture *_ctrl_arch, RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  myUtils::pretty_constructor(2, "SM: Double Support Swaying");

  ctrl_arch_ = _ctrl_arch;

  amp.setZero();
  freq.setZero();

  sp_ = DracoStateProvider::getStateProvider();
}

DoubleSupportSwaying::~DoubleSupportSwaying() {}

void DoubleSupportSwaying::firstVisit() {
  std::cout << "draco_states::kSwaying " << std::endl;

  ctrl_start_time_ = sp_->curr_time;

  ctrl_arch_->floating_base_tm->InitializeCoMSwayingTrajectory(sp_->curr_time,
                                                               amp, freq);
}

void DoubleSupportSwaying::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // Update Foot Task
  ctrl_arch_->rfoot_tm->useCurrent();
  ctrl_arch_->lfoot_tm->useCurrent();

  // Update Floating Base
  ctrl_arch_->floating_base_tm->UpdateFloatingBaseDesired(sp_->curr_time);
}

void DoubleSupportSwaying::lastVisit() {}

bool DoubleSupportSwaying::endOfState() { return false; }

StateIdentifier DoubleSupportSwaying::getNextState() {}
