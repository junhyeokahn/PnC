#include "PnC/draco_pnc/draco_state_machine/double_support_stand.hpp"

DoubleSupportStand::DoubleSupportStand(const StateIdentifier _state_identifier,
                                       DracoControlArchitecture *_ctrl_arch,
                                       RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  myUtils::pretty_constructor(2, "SM: Double Support Stand");

  ctrl_arch_ = _ctrl_arch;
  sp_ = DracoStateProvider::getStateProvider();
}

DoubleSupportStand::~DoubleSupportStand() {}

void DoubleSupportStand::firstVisit() {
  ctrl_start_time_ = sp_->curr_time;

  // Initialize CoM Trajectory
  Eigen::Isometry3d lfoot_iso = robot_->get_link_iso("l_foot_contact");
  Eigen::Isometry3d rfoot_iso = robot_->get_link_iso("r_foot_contact");
  Eigen::Vector3d target_com_pos =
      (lfoot_iso.translation() + rfoot_iso.translation()) / 2.;
  target_com_pos[2] = com_height_des;
  Eigen::Quaternion<double> target_base_ori =
      Eigen::Quaternion<double>(lfoot_iso.linear())
          .slerp(0.5, Eigen::Quaternion<double>(rfoot_iso.linear()));

  ctrl_arch_->floating_base_tm->InitializeFloatingBaseInterpolationTrajectory(
      sp_->curr_time, end_time, target_com_pos, target_base_ori);

  // Initialize Reaction Force Ramp to Max
  ctrl_arch_->rfoot_fm->InitializeRampToMax(sp_->curr_time, rf_z_max_time);
  ctrl_arch_->lfoot_fm->InitializeRampToMax(sp_->curr_time, rf_z_max_time);
}

void DoubleSupportStand::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // Update Floating Base Task
  ctrl_arch_->floating_base_tm->UpdateFloatingBaseDesired(sp_->curr_time);
  // Update Foot Task
  ctrl_arch_->rfoot_tm->useCurrent();
  ctrl_arch_->lfoot_tm->useCurrent();
  // Update Max Normal Reaction Force
  ctrl_arch_->rfoot_fm->UpdateRampToMax(sp_->curr_time);
  ctrl_arch_->lfoot_fm->UpdateRampToMax(sp_->curr_time);
}

void DoubleSupportStand::lastVisit() {}

bool DoubleSupportStand::endOfState() {
  if (state_machine_time_ > end_time) {
    return true;
  } else {
    return false;
  }
}

StateIdentifier DoubleSupportStand::getNextState() {
  return draco_states::kBalance;
}
