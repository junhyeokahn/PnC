#include "pnc/draco_pnc/draco_state_machine/contact_transition_end.hpp"

ContactTransitionEnd::ContactTransitionEnd(
    const StateIdentifier _state_identifier,
    DracoControlArchitecture *_ctrl_arch, int _leg_side, RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  util::PrettyConstructor(2, "ContactTransitionEnd");

  ctrl_arch_ = _ctrl_arch;
  leg_side_ = _leg_side;
  sp_ = DracoStateProvider::getStateProvider();
}

ContactTransitionEnd::~ContactTransitionEnd() {}

void ContactTransitionEnd::firstVisit() {
  if (leg_side_ == EndEffector::RFoot) {
    std::cout << "draco_states::kRFootContactTransitionEnd" << std::endl;
  } else {
    std::cout << "draco_states::kLFootContactTransitionEnd" << std::endl;
  }

  ctrl_start_time_ = sp_->curr_time;
  end_time_ = ctrl_arch_->dcm_tm->getNormalForceRampDownTime();

  if (leg_side_ == EndEffector::LFoot) {
    ctrl_arch_->lfoot_fm->InitializeRampToMin(sp_->curr_time, end_time_);
    ctrl_arch_->lfoot_pos_hm->InitializeRampToMin(sp_->curr_time, end_time_);
    ctrl_arch_->lfoot_ori_hm->InitializeRampToMin(sp_->curr_time, end_time_);
  } else if (leg_side_ == EndEffector::RFoot) {
    ctrl_arch_->rfoot_fm->InitializeRampToMin(sp_->curr_time, end_time_);
    ctrl_arch_->rfoot_pos_hm->InitializeRampToMin(sp_->curr_time, end_time_);
    ctrl_arch_->rfoot_ori_hm->InitializeRampToMin(sp_->curr_time, end_time_);
  } else {
    assert(false);
  }
}

void ContactTransitionEnd::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // Update max normal reaction forces & task hierarchy
  if (leg_side_ == EndEffector::LFoot) {
    ctrl_arch_->lfoot_fm->UpdateRampToMin(sp_->curr_time);
    ctrl_arch_->lfoot_pos_hm->UpdateRampToMin(sp_->curr_time);
    ctrl_arch_->lfoot_ori_hm->UpdateRampToMin(sp_->curr_time);
  } else if (leg_side_ == EndEffector::RFoot) {
    ctrl_arch_->rfoot_fm->UpdateRampToMin(sp_->curr_time);
    ctrl_arch_->rfoot_pos_hm->UpdateRampToMin(sp_->curr_time);
    ctrl_arch_->rfoot_ori_hm->UpdateRampToMin(sp_->curr_time);
  } else {
    assert(false);
  }

  // Update Foot Task
  ctrl_arch_->rfoot_tm->UseCurrent();
  ctrl_arch_->lfoot_tm->UseCurrent();

  // Update floating base task
  ctrl_arch_->dcm_tm->updateDCMTasksDesired(sp_->curr_time);
}

void ContactTransitionEnd::lastVisit() {}

bool ContactTransitionEnd::endOfState() {

  if (state_machine_time_ >= end_time_) {
    return true;
  } else {
    return false;
  }
}

StateIdentifier ContactTransitionEnd::getNextState() {
  if (leg_side_ == EndEffector::LFoot) {
    return draco_states::kLFootSwing;
  } else if (leg_side_ == EndEffector::RFoot) {
    return draco_states::kRFootSwing;
  }
}
