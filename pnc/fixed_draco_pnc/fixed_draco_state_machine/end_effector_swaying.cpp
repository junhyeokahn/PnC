#include "pnc/fixed_draco_pnc/fixed_draco_state_machine/end_effector_swaying.hpp"

EndEffectorSwaying::EndEffectorSwaying(
    const StateIdentifier _state_identifier,
    FixedDracoControlArchitecture *_ctrl_arch, int _leg_side,
    RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  util::PrettyConstructor(2, "EndEffectorSwaying");

  ctrl_arch_ = _ctrl_arch;
  leg_side_ = _leg_side;
  sp_ = FixedDracoStateProvider::getStateProvider();
}

EndEffectorSwaying::~EndEffectorSwaying() {}

void EndEffectorSwaying::firstVisit() {
  ctrl_start_time_ = sp_->curr_time;

  if (leg_side_ == EndEffector::RFoot) {
    std::cout << "fixed_draco_states::kRightFootSwaying" << std::endl;
    ctrl_arch_->rf_ee_tm->InitializeSwayingTrajectory(ctrl_start_time_, amp,
                                                      freq);
    target_lf_iso_ = robot_->get_link_iso("l_foot_contact");
  } else if (leg_side_ == EndEffector::LFoot) {
    std::cout << "fixed_draco_states::kLeftFootSwaying" << std::endl;
    ctrl_arch_->lf_ee_tm->InitializeSwayingTrajectory(ctrl_start_time_, amp,
                                                      freq);
    target_rf_iso_ = robot_->get_link_iso("r_foot_contact");
  } else {
    std::cout << "Wrong leg side" << std::endl;
  }
}

void EndEffectorSwaying::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  if (leg_side_ == EndEffector::RFoot) {
    ctrl_arch_->rf_ee_tm->UpdateDesired(sp_->curr_time);
    ctrl_arch_->lf_ee_tm->UpdateDesired(target_lf_iso_);
  } else if (leg_side_ == EndEffector::LFoot) {
    ctrl_arch_->lf_ee_tm->UpdateDesired(sp_->curr_time);
    ctrl_arch_->rf_ee_tm->UpdateDesired(target_rf_iso_);
  } else {
    std::cout << "Wrong leg side" << std::endl;
  }
}

void EndEffectorSwaying::lastVisit() {}

bool EndEffectorSwaying::endOfState() { return false; }

StateIdentifier EndEffectorSwaying::getNextState() {}
