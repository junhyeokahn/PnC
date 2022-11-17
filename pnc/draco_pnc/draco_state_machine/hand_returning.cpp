#include "pnc/draco_pnc/draco_state_machine/hand_returning.hpp"

HandReturning::HandReturning(const StateIdentifier _state_identifier_in,
                             DracoControlArchitecture *_ctrl_arch,
                             RobotSystem *_robot)
    : StateMachine(_state_identifier_in, _robot), duration_(0.),
      ctrl_start_time_(0.) {
  util::PrettyConstructor(2, "HandReturning");

  ctrl_arch_ = _ctrl_arch;
  sp_ = DracoStateProvider::getStateProvider();
}

void HandReturning::firstVisit() {
  ctrl_start_time_ = sp_->curr_time;

  if (state_identity_ == draco_states::kLHandReturning) {
    std::cout << "draco_states::kLHandReturning" << std::endl;
    ctrl_arch_->lhand_pos_hm->InitializeRampToMin(sp_->curr_time, duration_);
    ctrl_arch_->lhand_ori_hm->InitializeRampToMin(sp_->curr_time, duration_);

  } else if (state_identity_ == draco_states::kRHandReturning) {
    std::cout << "draco_states::kRHandReturning" << std::endl;
    ctrl_arch_->rhand_pos_hm->InitializeRampToMin(sp_->curr_time, duration_);
    ctrl_arch_->rhand_ori_hm->InitializeRampToMin(sp_->curr_time, duration_);

  } else
    std::cout << "[[Error]] no matching draco state machine" << std::endl;
}

void HandReturning::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  if (state_identity_ == draco_states::kLHandReturning) {
    ctrl_arch_->lhand_pos_hm->UpdateRampToMin(sp_->curr_time);
    ctrl_arch_->lhand_ori_hm->UpdateRampToMin(sp_->curr_time);
    ctrl_arch_->rfoot_tm->UpdateZeroAccCmd();
    ctrl_arch_->lfoot_tm->UpdateZeroAccCmd();
  } else if (state_identity_ == draco_states::kRHandReturning) {
    ctrl_arch_->rhand_pos_hm->UpdateRampToMin(sp_->curr_time);
    ctrl_arch_->rhand_ori_hm->UpdateRampToMin(sp_->curr_time);
    ctrl_arch_->rfoot_tm->UpdateZeroAccCmd();
    ctrl_arch_->lfoot_tm->UpdateZeroAccCmd();
  } else
    std::cout << "[[Error]] no matching draco state machine" << std::endl;
}

void HandReturning::lastVisit() {}

bool HandReturning::endOfState() {
  return state_machine_time_ > duration_ + 0.1 ? true : false;
}

StateIdentifier HandReturning::getNextState() { return draco_states::kBalance; }
