#include "pnc/draco_pnc/draco_state_machine/hand_reaching.hpp"

HandReaching::HandReaching(const StateIdentifier _state_identifier_in,
                           DracoControlArchitecture *_ctrl_arch,
                           RobotSystem *_robot)
    : StateMachine(_state_identifier_in, _robot), duration_(0.),
      rel_target_pos_(Eigen::Vector3d::Zero()),
      rel_target_ori_(Eigen::Quaterniond::Identity()),
      b_trigger_return_(false) {
  util::PrettyConstructor(2, "HandReaching");

  ctrl_arch_ = _ctrl_arch;
  sp_ = DracoStateProvider::getStateProvider();
}

void HandReaching::firstVisit() {
  Eigen::Vector3d target_pos = Eigen::Vector3d::Zero();
  Eigen::Quaterniond target_ori = Eigen::Quaterniond::Identity();

  if (state_identity_ == draco_states::kLHandReaching) {
    std::cout << "draco_states::kLHandReaching" << std::endl;
    target_pos =
        robot_->get_link_iso("l_hand_contact").translation() + rel_target_pos_;
    target_ori =
        Eigen::Quaterniond(robot_->get_link_iso("l_hand_contact").linear()) *
        rel_target_ori_;

    ctrl_arch_->lhand_tm->InitializeInterpolationTrajectory(
        sp_->curr_time, duration_, target_pos, target_ori);

    // initialize task hierarchy manager
    ctrl_arch_->lhand_pos_hm->InitializeRampToMax(sp_->curr_time,
                                                  duration_ / 2.);
    ctrl_arch_->lhand_ori_hm->InitializeRampToMax(sp_->curr_time,
                                                  duration_ / 2.);

  } else if (state_identity_ == draco_states::kRHandReaching) {
    std::cout << "draco_states::kRHandReaching" << std::endl;
    target_pos =
        robot_->get_link_iso("r_hand_contact").translation() + rel_target_pos_;
    target_ori =
        Eigen::Quaterniond(robot_->get_link_iso("r_hand_contact").linear()) *
        rel_target_ori_;

    ctrl_arch_->rhand_tm->InitializeInterpolationTrajectory(
        sp_->curr_time, duration_, target_pos, target_ori);

    // initialize task hierarchy manager
    ctrl_arch_->rhand_pos_hm->InitializeRampToMax(sp_->curr_time,
                                                  duration_ / 2.);
    ctrl_arch_->rhand_ori_hm->InitializeRampToMax(sp_->curr_time,
                                                  duration_ / 2.);
  } else
    std::cout << "[[Error]] no matching draco state machine" << std::endl;
}

void HandReaching::oneStep() {
  if (state_identity_ == draco_states::kLHandReaching) {
    ctrl_arch_->lhand_tm->UpdateDesired(sp_->curr_time);
    ctrl_arch_->lhand_pos_hm->UpdateRampToMax(sp_->curr_time);
    ctrl_arch_->lhand_ori_hm->UpdateRampToMax(sp_->curr_time);

  } else if (state_identity_ == draco_states::kRHandReaching) {
    ctrl_arch_->rhand_tm->UpdateDesired(sp_->curr_time);
    ctrl_arch_->rhand_pos_hm->UpdateRampToMax(sp_->curr_time);
    ctrl_arch_->rhand_ori_hm->UpdateRampToMax(sp_->curr_time);
  }
}

void HandReaching::lastVisit() {}

bool HandReaching::endOfState() { return b_trigger_return_ ? true : false; }

StateIdentifier HandReaching::getNextState() {
  // TODO:
  // adding hand return state
}
