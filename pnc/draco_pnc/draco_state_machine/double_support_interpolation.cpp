#include "pnc/draco_pnc/draco_state_machine/double_support_interpolation.hpp"

DoubleSupportInterpolation::DoubleSupportInterpolation(
    const StateIdentifier _state_identifier,
    DracoControlArchitecture *_ctrl_arch, RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  util::PrettyConstructor(2, "SM: Double Support Interpolation");

  ctrl_arch_ = _ctrl_arch;

  local_offset.setZero();
  end_time = 5.0;

  sp_ = DracoStateProvider::getStateProvider();
}

DoubleSupportInterpolation::~DoubleSupportInterpolation() {}

void DoubleSupportInterpolation::firstVisit() {
  std::cout << "draco_states::kBaseInterpolation" << std::endl;

  ctrl_start_time_ = sp_->curr_time;

  Eigen::Vector3d target_com_pos = robot_->get_com_pos();
  target_com_pos += local_offset;

  Eigen::Quaternion<double> target_base_quat = Eigen::Quaternion<double>(
      robot_->get_link_iso("torso_com_link").linear());

  ctrl_arch_->floating_base_tm->InitializeInterpolationTrajectory(
      sp_->curr_time, end_time, target_com_pos, target_base_quat);
}

void DoubleSupportInterpolation::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // Update Foot Task
  ctrl_arch_->rfoot_tm->UseCurrent();
  ctrl_arch_->lfoot_tm->UseCurrent();

  // Update Floating Base
  ctrl_arch_->floating_base_tm->UpdateDesired(sp_->curr_time);
}

void DoubleSupportInterpolation::lastVisit() {}

bool DoubleSupportInterpolation::endOfState() {
  if (state_machine_time_ > end_time) {
    return true;
  } else {
    return false;
  }
}

StateIdentifier DoubleSupportInterpolation::getNextState() {
  return draco_states::kBalance;
}
