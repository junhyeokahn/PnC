#include "pnc/draco_pnc/draco_state_machine/foot_lifting.hpp"

FootLifting::FootLifting(const StateIdentifier _state_identifier,
                         DracoControlArchitecture *_ctrl_arch, int _leg_side,
                         RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  util::PrettyConstructor(2, "FootLifting");

  ctrl_arch_ = _ctrl_arch;
  leg_side_ = _leg_side;
  sp_ = DracoStateProvider::getStateProvider();
  ramp_time_ = 0.5;
}

FootLifting::~FootLifting() {}

void FootLifting::firstVisit() {
  sp_->b_lf_contact = true;
  sp_->b_rf_contact = true;

  ctrl_start_time_ = sp_->curr_time;

  if (leg_side_ == EndEffector::RFoot) {
//    std::cout << "draco_states::kRFootLifting[transition]" << std::endl;

    // force manager
    ctrl_arch_->rfoot_fm->InitializeRampToMin(sp_->curr_time, ramp_time_);
    ctrl_arch_->lfoot_fm->InitializeRampToMax(sp_->curr_time, ramp_time_);

    // hierarchy manager
    ctrl_arch_->rfoot_pos_hm->InitializeRampToMin(sp_->curr_time, ramp_time_);
    ctrl_arch_->rfoot_ori_hm->InitializeRampToMin(sp_->curr_time, ramp_time_);
    ctrl_arch_->lfoot_pos_hm->InitializeRampToMax(sp_->curr_time, ramp_time_);
    ctrl_arch_->lfoot_ori_hm->InitializeRampToMax(sp_->curr_time, ramp_time_);
  } else {
    std::cout << "draco_states::kLFootLifting[transition]" << std::endl;

    // force manager
    ctrl_arch_->rfoot_fm->InitializeRampToMax(sp_->curr_time, ramp_time_);
    ctrl_arch_->lfoot_fm->InitializeRampToMin(sp_->curr_time, ramp_time_);

    // hierarchy manager
    ctrl_arch_->rfoot_pos_hm->InitializeRampToMax(sp_->curr_time, ramp_time_);
    ctrl_arch_->rfoot_ori_hm->InitializeRampToMax(sp_->curr_time, ramp_time_);
    ctrl_arch_->lfoot_pos_hm->InitializeRampToMin(sp_->curr_time, ramp_time_);
    ctrl_arch_->lfoot_ori_hm->InitializeRampToMin(sp_->curr_time, ramp_time_);
  }
}

void FootLifting::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  if (leg_side_ == EndEffector::RFoot) {
    // Update max normal reaction forces
    ctrl_arch_->rfoot_fm->UpdateRampToMin(sp_->curr_time);
    ctrl_arch_->lfoot_fm->UpdateRampToMax(sp_->curr_time);

    // Update task hierarchy
    ctrl_arch_->rfoot_pos_hm->UpdateRampToMin(sp_->curr_time);
    ctrl_arch_->rfoot_ori_hm->UpdateRampToMin(sp_->curr_time);
    ctrl_arch_->lfoot_pos_hm->UpdateRampToMax(sp_->curr_time);
    ctrl_arch_->lfoot_ori_hm->UpdateRampToMax(sp_->curr_time);

  } else if (leg_side_ == EndEffector::LFoot) {
    // Update max normal reaction forces
    ctrl_arch_->rfoot_fm->UpdateRampToMax(sp_->curr_time);
    ctrl_arch_->lfoot_fm->UpdateRampToMin(sp_->curr_time);

    // Update task hierarchy
    ctrl_arch_->rfoot_pos_hm->UpdateRampToMax(sp_->curr_time);
    ctrl_arch_->rfoot_ori_hm->UpdateRampToMax(sp_->curr_time);
    ctrl_arch_->lfoot_pos_hm->UpdateRampToMin(sp_->curr_time);
    ctrl_arch_->lfoot_ori_hm->UpdateRampToMin(sp_->curr_time);
  } else {
    assert(false);
  }

  // Update Foot Task
  ctrl_arch_->rfoot_tm->UpdateZeroAccCmd();
  ctrl_arch_->lfoot_tm->UpdateZeroAccCmd();

  // Update floating base traj
}

void FootLifting::lastVisit() {}

bool FootLifting::endOfState() {
  if (state_machine_time_ >= ramp_time_) {
    return true;
  } else {
    return false;
  }
}

StateIdentifier FootLifting::getNextState() {
  if (b_step_by_step) {
    if (leg_side_ == EndEffector::LFoot) {
      return draco_states::kLFootSingleSupportLifting;
    } else if (leg_side_ == EndEffector::RFoot) {
      return draco_states::kRFootSingleSupportLifting;
    }
  } else {
    if (leg_side_ == EndEffector::LFoot) {
      return draco_states::kLFootSwingStatic;
    } else if (leg_side_ == EndEffector::RFoot) {
      return draco_states::kRFootSwingStatic;
    }
  }
}
