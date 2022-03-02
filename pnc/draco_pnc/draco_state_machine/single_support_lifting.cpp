#include "pnc/draco_pnc/draco_state_machine/single_support_lifting.hpp"

SingleSupportLifting::SingleSupportLifting(
    const StateIdentifier _state_identifier,
    DracoControlArchitecture *_ctrl_arch, int _leg_side, RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  util::PrettyConstructor(2, "SingleSupportLifting");

  ctrl_arch_ = _ctrl_arch;
  leg_side_ = _leg_side;
  sp_ = DracoStateProvider::getStateProvider();

  b_static_balancing_trigger = false;

  moving_duration_ = 1.;
  des_foot_z_pos_ = 0.5;
}

SingleSupportLifting::~SingleSupportLifting() {}

void SingleSupportLifting::firstVisit() {
  ctrl_start_time_ = sp_->curr_time;
  if (leg_side_ == EndEffector::RFoot) {
    std::cout << "draco_states::kRFootSingleSupportLifting" << std::endl;

    sp_->b_rf_contact = false;
    sp_->b_lf_contact = true;

    Eigen::Vector3d des_foot_pos =
        robot_->get_link_iso("r_foot_contact").translation();
    des_foot_pos[2] = des_foot_z_pos_;

    Eigen::Quaternion<double> des_foot_ori(
        robot_->get_link_iso("r_foot_contact").linear());

    ctrl_arch_->rfoot_tm->InitializeInterpolationTrajectory(
        sp_->curr_time, moving_duration_, des_foot_pos, des_foot_ori);

  } else if (leg_side_ == EndEffector::LFoot) {
    std::cout << "draco_states::kLFootSingleSupportLifting" << std::endl;

    sp_->b_rf_contact = true;
    sp_->b_lf_contact = false;

    Eigen::Vector3d des_foot_pos =
        robot_->get_link_iso("l_foot_contact").translation();
    des_foot_pos[2] = des_foot_z_pos_;

    Eigen::Quaternion<double> des_foot_ori(
        robot_->get_link_iso("l_foot_contact").linear());

    ctrl_arch_->lfoot_tm->InitializeInterpolationTrajectory(
        sp_->curr_time, moving_duration_, des_foot_pos, des_foot_ori);
  } else {
    assert(false);
  }
}

void SingleSupportLifting::oneStep() {
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

void SingleSupportLifting::lastVisit() {}

bool SingleSupportLifting::endOfState() {
  if (state_machine_time_ >= moving_duration_ && b_static_balancing_trigger) {
    return true;
  } else {
    return false;
  }
}

StateIdentifier SingleSupportLifting::getNextState() {
  if (leg_side_ == EndEffector::LFoot) {
    return draco_states::kLFootSingleSupportLanding;
  } else {
    return draco_states::kRFootSingleSupportLanding;
  }
}
