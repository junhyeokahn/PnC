#include <pnc/atlas_pnc/atlas_state_machine/contact_transition_end.hpp>

ContactTransitionEnd::ContactTransitionEnd(
    const StateIdentifier _state_identifier,
    AtlasControlArchitecture *_ctrl_arch, int _leg_side, RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  util::PrettyConstructor(2, "ContactTransitionEnd");

  atlas_ctrl_arch_ = _ctrl_arch;
  leg_side_ = _leg_side;
  sp_ = AtlasStateProvider::getStateProvider(_robot);
}

ContactTransitionEnd::~ContactTransitionEnd() {}

void ContactTransitionEnd::firstVisit() {
  if (leg_side_ == EndEffector::RFoot) {
    std::cout << "AtlasState::RFootContactTransitionEnd" << std::endl;
  } else {
    std::cout << "AtlasState::LFootContactTransitionEnd" << std::endl;
  }

  ctrl_start_time_ = sp_->curr_time;
  end_time_ = atlas_ctrl_arch_->dcm_tm->getNormalForceRampDownTime();

  if (leg_side_ == EndEffector::LFoot) {
    atlas_ctrl_arch_->lfoot_fm->InitializeRampToMin(sp_->curr_time, end_time_);
    atlas_ctrl_arch_->lfoot_pos_hm->InitializeRampToMin(sp_->curr_time,
                                                        end_time_);
    atlas_ctrl_arch_->lfoot_ori_hm->InitializeRampToMin(sp_->curr_time,
                                                        end_time_);
  } else if (leg_side_ == EndEffector::RFoot) {
    atlas_ctrl_arch_->rfoot_fm->InitializeRampToMin(sp_->curr_time, end_time_);
    atlas_ctrl_arch_->rfoot_pos_hm->InitializeRampToMin(sp_->curr_time,
                                                        end_time_);
    atlas_ctrl_arch_->rfoot_ori_hm->InitializeRampToMin(sp_->curr_time,
                                                        end_time_);
  } else {
    assert(false);
  }
}

void ContactTransitionEnd::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // Update max normal reaction forces & task hierarchy
  if (leg_side_ == EndEffector::LFoot) {
    atlas_ctrl_arch_->lfoot_fm->UpdateRampToMin(sp_->curr_time);
    atlas_ctrl_arch_->lfoot_pos_hm->UpdateRampToMin(sp_->curr_time);
    atlas_ctrl_arch_->lfoot_ori_hm->UpdateRampToMin(sp_->curr_time);
  } else if (leg_side_ == EndEffector::RFoot) {
    atlas_ctrl_arch_->rfoot_fm->UpdateRampToMin(sp_->curr_time);
    atlas_ctrl_arch_->rfoot_pos_hm->UpdateRampToMin(sp_->curr_time);
    atlas_ctrl_arch_->rfoot_ori_hm->UpdateRampToMin(sp_->curr_time);
  } else {
    assert(false);
  }

  // Update Foot Task
  atlas_ctrl_arch_->rfoot_tm->useCurrent();
  atlas_ctrl_arch_->lfoot_tm->useCurrent();

  // Update floating base task
  atlas_ctrl_arch_->dcm_tm->updateDCMTasksDesired(sp_->curr_time);
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
    return AtlasStates::LFootSwing;
  } else if (leg_side_ == EndEffector::RFoot) {
    return AtlasStates::RFootSwing;
  }
}
