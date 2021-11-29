#include "pnc/draco_pnc/draco_state_machine/double_support_balance.hpp"

DoubleSupportBalance::DoubleSupportBalance(
    const StateIdentifier _state_identifier,
    DracoControlArchitecture *_ctrl_arch, RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  util::PrettyConstructor(2, "DoubleSupportBalance");

  ctrl_arch_ = _ctrl_arch;
  sp_ = DracoStateProvider::getStateProvider();

  b_walking_trigger = false;
  b_swaying_trigger = false;
  b_interpolation_trigger = false;
  b_static_walking_trigger = false;
}

DoubleSupportBalance::~DoubleSupportBalance() {}

void DoubleSupportBalance::firstVisit() {
  std::cout << "draco_states::kBalance" << std::endl;

  ctrl_start_time_ = sp_->curr_time;
  b_walking_trigger = false;
  b_swaying_trigger = false;
  b_interpolation_trigger = false;
  b_static_walking_trigger = false;

  Eigen::Isometry3d lfoot_iso = robot_->get_link_iso("l_foot_contact");
  Eigen::Isometry3d rfoot_iso = robot_->get_link_iso("r_foot_contact");
}

void DoubleSupportBalance::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // Update Foot Task
  ctrl_arch_->rfoot_tm->UpdateZeroAccCmd();
  ctrl_arch_->lfoot_tm->UpdateZeroAccCmd();
}

void DoubleSupportBalance::lastVisit() {

  std::cout << "Base height" << std::endl;
  std::cout << robot_->get_link_iso("torso_com_link").translation()[2]
            << std::endl;
}

bool DoubleSupportBalance::endOfState() {

  if ((b_walking_trigger) && (ctrl_arch_->dcm_tm->footstep_list.size() > 0) &&
      (!(ctrl_arch_->dcm_tm->noRemainingSteps()))) {
    return true;
  }
  if (b_swaying_trigger) {
    return true;
  }
  if (b_interpolation_trigger) {
    return true;
  }

  if (b_static_walking_trigger) {
    return true;
  }
  return false;
}

StateIdentifier DoubleSupportBalance::getNextState() {
  int robot_side;
  if (ctrl_arch_->dcm_tm->nextStepRobotSide(robot_side)) {
    if (robot_side == EndEffector::LFoot) {
      return draco_states::kLFootContactTransitionStart;
    } else {
      return draco_states::kRFootContactTransitionStart;
    }
  }

  if (b_swaying_trigger) {
    return draco_states::kSwaying;
  }

  if (b_interpolation_trigger) {
    return draco_states::kBaseInterpolation;
  }

  // Static walking
  if (b_static_walking_trigger) {
    if (sp_->stance_foot == "l_foot_contact") {
      return draco_states::kMoveCoMToLFoot;
    } else if (sp_->stance_foot == "r_foot_contact") {
      return draco_states::kMoveCoMToRFoot;
    } else {
      assert(false);
    }
  }
}
