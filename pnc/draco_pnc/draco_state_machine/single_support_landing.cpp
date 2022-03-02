#include "pnc/draco_pnc/draco_state_machine/single_support_landing.hpp"

SingleSupportLanding::SingleSupportLanding(
    const StateIdentifier _state_identifier,
    DracoControlArchitecture *_ctrl_arch, int _leg_side, RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  util::PrettyConstructor(2, "SingleSupportLanding");

  ctrl_arch_ = _ctrl_arch;
  leg_side_ = _leg_side;
  sp_ = DracoStateProvider::getStateProvider();

  moving_duration_ = 1.;
}

SingleSupportLanding::~SingleSupportLanding() {}

void SingleSupportLanding::firstVisit() {
  ctrl_start_time_ = sp_->curr_time;
  if (leg_side_ == EndEffector::RFoot) {
    std::cout << "draco_states::kRFootSingleSupportLanding" << std::endl;

    Eigen::Vector3d des_foot_pos =
        robot_->get_link_iso("r_foot_contact").translation();
    des_foot_pos[2] = 0.;

    Eigen::Quaternion<double> des_foot_ori(
        robot_->get_link_iso("r_foot_contact").linear());

    ctrl_arch_->rfoot_tm->InitializeInterpolationTrajectory(
        sp_->curr_time, moving_duration_, des_foot_pos, des_foot_ori);

  } else if (leg_side_ == EndEffector::LFoot) {
    std::cout << "draco_states::kLFootSingleSupportLanding" << std::endl;

    Eigen::Vector3d des_foot_pos =
        robot_->get_link_iso("l_foot_contact").translation();
    des_foot_pos[2] = 0.;

    Eigen::Quaternion<double> des_foot_ori(
        robot_->get_link_iso("l_foot_contact").linear());

    ctrl_arch_->lfoot_tm->InitializeInterpolationTrajectory(
        sp_->curr_time, moving_duration_, des_foot_pos, des_foot_ori);
  } else {
    assert(false);
  }
}

void SingleSupportLanding::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // Update Foot Task
  if (leg_side_ == EndEffector::LFoot) {
    ctrl_arch_->lfoot_tm->UpdateInterpolationDesired(sp_->curr_time);
    ctrl_arch_->rfoot_tm->UpdateZeroAccCmd();
  } else {
    ctrl_arch_->rfoot_tm->UpdateInterpolationDesired(sp_->curr_time);
    ctrl_arch_->lfoot_tm->UpdateZeroAccCmd();
  }
}

void SingleSupportLanding::lastVisit() {}

bool SingleSupportLanding::endOfState() {
  if (state_machine_time_ >= moving_duration_) {
    return true;
  } else {
    return false;
  }
}

StateIdentifier SingleSupportLanding::getNextState() {
  if (leg_side_ == EndEffector::LFoot) {
    return draco_states::kLFootLanding;
  } else {
    return draco_states::kRFootLanding;
  }
}
