#include "pnc/fixed_draco_pnc/fixed_draco_state_machine/end_effector_hold.hpp"

EndEffectorHold::EndEffectorHold(const StateIdentifier _state_identifier,
                                 FixedDracoControlArchitecture *_ctrl_arch,
                                 RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  util::PrettyConstructor(2, "EndEffectorHold");

  ctrl_arch_ = _ctrl_arch;

  b_rf_swaying_trigger = false;
  b_lf_swaying_trigger = false;

  sp_ = FixedDracoStateProvider::getStateProvider();
}

EndEffectorHold::~EndEffectorHold() {}

void EndEffectorHold::firstVisit() {
  std::cout << "fixed_draco_states::kHold" << std::endl;
  ctrl_start_time_ = sp_->curr_time;
  target_rf_iso_ = robot_->get_link_iso("r_foot_contact");
  target_lf_iso_ = robot_->get_link_iso("l_foot_contact");
  b_rf_swaying_trigger = false;
  b_lf_swaying_trigger = false;
}

void EndEffectorHold::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  ctrl_arch_->rf_ee_tm->UpdateDesired(target_rf_iso_);
  ctrl_arch_->lf_ee_tm->UpdateDesired(target_lf_iso_);
}

void EndEffectorHold::lastVisit() {}

bool EndEffectorHold::endOfState() {
  if (b_rf_swaying_trigger || b_lf_swaying_trigger) {
    return true;
  } else {
    return false;
  }
}

StateIdentifier EndEffectorHold::getNextState() {
  if (b_rf_swaying_trigger) {
    return fixed_draco_states::kRightFootSwaying;
  } else if (b_lf_swaying_trigger) {
    return fixed_draco_states::kLeftFootSwaying;
  } else {
    assert(false);
  }
}
