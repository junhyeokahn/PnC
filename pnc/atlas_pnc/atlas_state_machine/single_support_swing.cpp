#include <pnc/atlas_pnc/atlas_state_machine/single_support_swing.hpp>

SingleSupportSwing::SingleSupportSwing(const StateIdentifier _state_identifier,
                                       AtlasControlArchitecture *_ctrl_arch,
                                       int _leg_side, RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  util::PrettyConstructor(2, "SingleSupportSwing");

  atlas_ctrl_arch_ = _ctrl_arch;
  leg_side_ = _leg_side;
  sp_ = AtlasStateProvider::getStateProvider(_robot);
}

SingleSupportSwing::~SingleSupportSwing() {}

void SingleSupportSwing::firstVisit() {
  if (leg_side_ == EndEffector::RFoot) {
    std::cout << "AtlasState::RFootSwing" << std::endl;
  } else {
    std::cout << "AtlasState::LFootSwing" << std::endl;
  }

  ctrl_start_time_ = sp_->curr_time;
  end_time_ = atlas_ctrl_arch_->dcm_tm->getSwingTime();

  int footstep_idx = atlas_ctrl_arch_->dcm_tm->current_footstep_idx;

  if (leg_side_ == EndEffector::RFoot) {
    atlas_ctrl_arch_->rfoot_tm->InitializeSwingTrajectory(
        sp_->curr_time, end_time_,
        atlas_ctrl_arch_->dcm_tm->footstep_list[footstep_idx]);
  } else if (leg_side_ == EndEffector::LFoot) {
    atlas_ctrl_arch_->lfoot_tm->InitializeSwingTrajectory(
        sp_->curr_time, end_time_,
        atlas_ctrl_arch_->dcm_tm->footstep_list[footstep_idx]);
  } else {
    assert(false);
  }
}

void SingleSupportSwing::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // Update Foot Task
  if (leg_side_ == EndEffector::LFoot) {
    atlas_ctrl_arch_->lfoot_tm->UpdateDesired(sp_->curr_time);
    atlas_ctrl_arch_->rfoot_tm->UpdateZeroAccCmd();
  } else {
    atlas_ctrl_arch_->rfoot_tm->UpdateDesired(sp_->curr_time);
    atlas_ctrl_arch_->lfoot_tm->UpdateZeroAccCmd();
  }

  // Update floating base task
  atlas_ctrl_arch_->dcm_tm->updateDCMTasksDesired(sp_->curr_time);
}

void SingleSupportSwing::lastVisit() {
  atlas_ctrl_arch_->dcm_tm->incrementStepIndex();
}

bool SingleSupportSwing::endOfState() {

  // if (state_machine_time_ >= end_time_) {
  // return true;
  //} else {
  // if (state_machine_time_ >= 0.5 * end_time_) {
  // if (leg_side_ == EndEffector::LFoot) {
  // if (sp_->b_lf_contact) {
  // printf("Early left foot contact at %f/%f\n", state_machine_time_,
  // end_time_);
  // return true;
  //}
  //} else {
  // if (sp_->b_rf_contact) {
  // printf("Early right foot contact at %f/%f\n", state_machine_time_,
  // end_time_);
  // return true;
  //}
  //}
  //}
  // return false;
  //}

  if (state_machine_time_ >= end_time_) {
    return true;
  } else {
    return false;
  }
}

StateIdentifier SingleSupportSwing::getNextState() {
  int next_footstep_robot_side;
  if (atlas_ctrl_arch_->dcm_tm->nextStepRobotSide(next_footstep_robot_side)) {
    if (next_footstep_robot_side == EndEffector::LFoot) {
      return AtlasStates::LFootContactTransitionStart;
    } else {
      return AtlasStates::RFootContactTransitionStart;
    }
  } else {
    if (leg_side_ == EndEffector::LFoot) {
      return AtlasStates::LFootContactTransitionStart;
    } else {
      return AtlasStates::RFootContactTransitionStart;
    }
  }
}
