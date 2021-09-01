#include "pnc/draco_pnc/draco_state_machine/double_support_move.hpp"

DoubleSupportMove::DoubleSupportMove(const StateIdentifier _state_identifier,
                                     DracoControlArchitecture *_ctrl_arch,
                                     int _com_move_states, RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  util::PrettyConstructor(2, "DoubleSupportMove");

  ctrl_arch_ = _ctrl_arch;
  sp_ = DracoStateProvider::getStateProvider();
  com_move_states_ = _com_move_states;
}

DoubleSupportMove::~DoubleSupportMove() {}

void DoubleSupportMove::firstVisit() {

  if (com_move_states_ == com_move_states::Left) {
    std::cout << "draco_states::kMoveCoMToLFoot" << std::endl;

    Eigen::Vector3d target_com_pos =
        robot_->get_link_iso("l_foot_contact").translation();
    target_com_pos[2] = des_com_height_;
    Eigen::Quaternion<double> target_base_quat = Eigen::Quaternion<double>(
        robot_->get_link_iso("torso_com_link").linear());

    ctrl_arch_->floating_base_tm->InitializeInterpolationTrajectory(
        sp_->curr_time, moving_duration_, target_com_pos, target_base_quat);

  } else if (com_move_states_ == com_move_states::Right) {
    std::cout << "draco_states::kMoveCoMToRFoot" << std::endl;

    Eigen::Vector3d target_com_pos =
        robot_->get_link_iso("r_foot_contact").translation();
    target_com_pos[2] = des_com_height_;
    Eigen::Quaternion<double> target_base_quat = Eigen::Quaternion<double>(
        robot_->get_link_iso("torso_com_link").linear());

    ctrl_arch_->floating_base_tm->InitializeInterpolationTrajectory(
        sp_->curr_time, moving_duration_, target_com_pos, target_base_quat);

  } else if (com_move_states_ == com_move_states::Center) {
    std::cout << "draco_states::kMoveComToCenter" << std::endl;
    Eigen::Vector3d target_com_pos =
        0.5 * (robot_->get_link_iso("l_foot_contact").translation() +
               robot_->get_link_iso("r_foot_contact").translation());
    target_com_pos[2] = des_com_height_;
    Eigen::Quaternion<double> left_foot_ori(
        robot_->get_link_iso("l_foot_contact").linear());
    Eigen::Quaternion<double> right_foot_ori(
        robot_->get_link_iso("r_foot_contact").linear());
    Eigen::Quaternion<double> target_base_quat =
        left_foot_ori.slerp(0.5, right_foot_ori);

    ctrl_arch_->floating_base_tm->InitializeInterpolationTrajectory(
        sp_->curr_time, moving_duration_, target_com_pos, target_base_quat);
  } else {
    std::cout << "invalid draco_states" << std::endl;
  }

  ctrl_start_time_ = sp_->curr_time;
}

void DoubleSupportMove::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // Update Foot Task
  ctrl_arch_->rfoot_tm->UpdateZeroAccCmd();
  ctrl_arch_->lfoot_tm->UpdateZeroAccCmd();

  // Update Floating Base
  ctrl_arch_->floating_base_tm->UpdateDesired(sp_->curr_time);
}

void DoubleSupportMove::lastVisit() {}

bool DoubleSupportMove::endOfState() {

  if (state_machine_time_ > moving_duration_) {
    return true;
  }
  return false;
}

StateIdentifier DoubleSupportMove::getNextState() {
  if (com_move_states_ == com_move_states::Left) {
    return draco_states::kRFootLifting;
  }
  if (com_move_states_ == com_move_states::Right) {
    return draco_states::kLFootLifting;
  }
  if (com_move_states_ == com_move_states::Center) {
    return draco_states::kBalance;
  }
}
