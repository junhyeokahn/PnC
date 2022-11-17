#include "pnc/draco_pnc/draco_state_machine/foot_landing.hpp"

FootLanding::FootLanding(const StateIdentifier _state_identifier,
                         DracoControlArchitecture *_ctrl_arch, int _leg_side,
                         RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  util::PrettyConstructor(2, "FootLanding");

  ctrl_arch_ = _ctrl_arch;
  leg_side_ = _leg_side;
  sp_ = DracoStateProvider::getStateProvider();
  ramp_time_ = 0.5;

  b_static_walking_trigger = false;
}

FootLanding::~FootLanding() {}

void FootLanding::firstVisit() {
  sp_->b_lf_contact = true;
  sp_->b_rf_contact = true;

  ctrl_start_time_ = sp_->curr_time;

  if (leg_side_ == EndEffector::RFoot) {
    std::cout << "draco_states::kRFootLanding" << std::endl;
  } else {
    std::cout << "draco_states::kLFootLanding" << std::endl;
  }
  // force manager
  ctrl_arch_->rfoot_fm->InitializeRampToMax(sp_->curr_time, ramp_time_);
  ctrl_arch_->lfoot_fm->InitializeRampToMax(sp_->curr_time, ramp_time_);

  // hierarchy manager
  ctrl_arch_->rfoot_pos_hm->InitializeRampToMax(sp_->curr_time, ramp_time_);
  ctrl_arch_->rfoot_ori_hm->InitializeRampToMax(sp_->curr_time, ramp_time_);
  ctrl_arch_->lfoot_pos_hm->InitializeRampToMax(sp_->curr_time, ramp_time_);
  ctrl_arch_->lfoot_ori_hm->InitializeRampToMax(sp_->curr_time, ramp_time_);
}

void FootLanding::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // Update max normal reaction forces
  ctrl_arch_->rfoot_fm->UpdateRampToMax(sp_->curr_time);
  ctrl_arch_->lfoot_fm->UpdateRampToMax(sp_->curr_time);

  // Update task hierarchy
  ctrl_arch_->rfoot_pos_hm->UpdateRampToMax(sp_->curr_time);
  ctrl_arch_->rfoot_ori_hm->UpdateRampToMax(sp_->curr_time);
  ctrl_arch_->lfoot_pos_hm->UpdateRampToMax(sp_->curr_time);
  ctrl_arch_->lfoot_ori_hm->UpdateRampToMax(sp_->curr_time);

  // Update Foot Task
  ctrl_arch_->rfoot_tm->UpdateZeroAccCmd();
  ctrl_arch_->lfoot_tm->UpdateZeroAccCmd();
  // ctrl_arch_->rfoot_tm->useNominalPoseCmd(sp_->nominal_rfoot_iso);
  // ctrl_arch_->lfoot_tm->useNominalPoseCmd(sp_->nominal_lfoot_iso);

  // Update floating base traj
}

void FootLanding::lastVisit() { b_static_walking_trigger = false; }

bool FootLanding::endOfState() {
  if (state_machine_time_ >= ramp_time_ && b_static_walking_trigger) {
    return true;
  } else {
    return false;
  }
}

StateIdentifier FootLanding::getNextState() {
  if (leg_side_ == EndEffector::LFoot) {
    sp_->stance_foot = "l_foot_contact";
  } else if (leg_side_ == EndEffector::RFoot) {
    sp_->stance_foot = "r_foot_contact";
  }
  return draco_states::kMoveCoMToCenter;
}
