#include "pnc/draco_pnc/draco_state_machine/double_support_move.hpp"

DoubleSupportMove::DoubleSupportMove(const StateIdentifier _state_identifier,
                                     DracoControlArchitecture *_ctrl_arch,
                                     int _com_move_states, RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  util::PrettyConstructor(2, "DoubleSupportMove");

  ctrl_arch_ = _ctrl_arch;
  sp_ = DracoStateProvider::getStateProvider();
  com_move_states_ = _com_move_states;
  b_static_walking_trigger = false;
}

DoubleSupportMove::~DoubleSupportMove() {}

void DoubleSupportMove::firstVisit() {

  ctrl_start_time_ = sp_->curr_time;

  sp_->b_lf_contact = true;
  sp_->b_rf_contact = true;

  if (com_move_states_ == com_move_states::Left) {
    std::cout << "draco_states::kMoveCoMToLFoot" << std::endl;

    Eigen::Vector3d target_com_pos =
        robot_->get_link_iso("l_foot_contact").translation();
    target_com_pos[2] = des_com_height_;
    Eigen::Quaternion<double> target_base_quat = sp_->nominal_base_quat;

    Eigen::Vector2d global_com_offset =
        target_base_quat.toRotationMatrix().block(0, 0, 2, 2) * -com_offset;
    target_com_pos[0] += global_com_offset[0];
    target_com_pos[1] += global_com_offset[1];

    ctrl_arch_->floating_base_tm->InitializeInterpolationTrajectory(
        sp_->curr_time, moving_duration_, target_com_pos, target_base_quat);

  } else if (com_move_states_ == com_move_states::Right) {
    std::cout << "draco_states::kMoveCoMToRFoot" << std::endl;

    Eigen::Vector3d target_com_pos =
        robot_->get_link_iso("r_foot_contact").translation();

    target_com_pos[2] = des_com_height_;
    Eigen::Quaternion<double> target_base_quat = sp_->nominal_base_quat;

    Eigen::Vector2d global_com_offset =
        target_base_quat.toRotationMatrix().block(0, 0, 2, 2) * com_offset;
    target_com_pos[0] += global_com_offset[0];
    target_com_pos[1] += global_com_offset[1];

    // std::cout << "r_foot_contact pos: "
    //<< robot_->get_link_iso("r_foot_contact").translation()
    //<< std::endl;
    // std::cout << "target com pos: " << target_com_pos << std::endl;
    // std::cout << "target_base_quat: " << target_base_quat.toRotationMatrix()
    //<< std::endl;

    ctrl_arch_->floating_base_tm->InitializeInterpolationTrajectory(
        sp_->curr_time, moving_duration_, target_com_pos, target_base_quat);

  } else if (com_move_states_ == com_move_states::Center) {
    std::cout << "draco_states::kMoveComToCenter" << std::endl;
    Eigen::Vector3d target_com_pos =
        0.5 * (robot_->get_link_iso("l_foot_contact").translation() +
               robot_->get_link_iso("r_foot_contact").translation());
    target_com_pos[2] = des_com_height_;
    Eigen::Quaternion<double> target_base_quat = sp_->nominal_base_quat;

    ctrl_arch_->floating_base_tm->InitializeInterpolationTrajectory(
        sp_->curr_time, moving_duration_, target_com_pos, target_base_quat);
  } else {
    std::cout << "invalid draco_states" << std::endl;
  }
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

  // if (state_machine_time_ > moving_duration_) {
  // return true;
  //}
  if (state_machine_time_ > moving_duration_ && b_static_walking_trigger) {
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
