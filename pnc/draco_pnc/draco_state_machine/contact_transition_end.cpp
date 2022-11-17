#include "pnc/draco_pnc/draco_state_machine/contact_transition_end.hpp"

ContactTransitionEnd::ContactTransitionEnd(
    const StateIdentifier _state_identifier,
    DracoControlArchitecture *_ctrl_arch, int _leg_side, RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  util::PrettyConstructor(2, "ContactTransitionEnd");

  ctrl_arch_ = _ctrl_arch;
  leg_side_ = _leg_side;
  sp_ = DracoStateProvider::getStateProvider();
  contact_detection_manager_ = new ContactDetectionManager(_robot, "l_foot_contact",
                                               "r_foot_contact");
  has_swing_foot_touochdown_ = false;
}

ContactTransitionEnd::~ContactTransitionEnd() {delete contact_detection_manager_;}

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

  contact_detection_manager_->update_contact_stance(leg_side_);
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
  ctrl_arch_->rfoot_tm->UpdateZeroAccCmd();
  ctrl_arch_->lfoot_tm->UpdateZeroAccCmd();
  // ctrl_arch_->rfoot_tm->useNominalPoseCmd(sp_->nominal_rfoot_iso);
  // ctrl_arch_->lfoot_tm->useNominalPoseCmd(sp_->nominal_lfoot_iso);

  // Update floating base task
  ctrl_arch_->dcm_tm->updateDCMTasksDesired(sp_->curr_time);

  double expected_height_difference = 0.;
  has_swing_foot_touochdown_ = contact_detection_manager_->check_swing_foot_contact(expected_height_difference);
}

void ContactTransitionEnd::lastVisit() {
  sp_->nominal_lfoot_iso.translation() = ctrl_arch_->lfoot_tm->GetDesiredPos();
  sp_->nominal_lfoot_iso.linear() =
      ctrl_arch_->lfoot_tm->GetDesiredOri().normalized().toRotationMatrix();
  sp_->nominal_rfoot_iso.translation() = ctrl_arch_->rfoot_tm->GetDesiredPos();
  sp_->nominal_rfoot_iso.linear() =
      ctrl_arch_->rfoot_tm->GetDesiredOri().normalized().toRotationMatrix();
}

bool ContactTransitionEnd::endOfState() {
  if ((state_machine_time_ >= end_time_) || has_swing_foot_touochdown_) {
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
