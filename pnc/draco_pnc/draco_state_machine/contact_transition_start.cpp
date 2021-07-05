#include "pnc/draco_pnc/draco_state_machine/contact_transition_start.hpp"

ContactTransitionStart::ContactTransitionStart(
    const StateIdentifier _state_identifier,
    DracoControlArchitecture *_ctrl_arch, int _leg_side, RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  util::PrettyConstructor(2, "SM: Double Support Balance");

  ctrl_arch_ = _ctrl_arch;
  leg_side_ = _leg_side;
  sp_ = DracoStateProvider::getStateProvider();
}

ContactTransitionStart::~ContactTransitionStart() {}

void ContactTransitionStart::firstVisit() {
  if (leg_side_ == EndEffector::RFoot) {
    std::cout << "draco_states::kRFootContactTransitionStart" << std::endl;
  } else {
    std::cout << "draco_states::kLFootContactTransitionStart" << std::endl;
  }

  ctrl_start_time_ = sp_->curr_time;

  // Initialize Reaction Force Ramp to Max
  ctrl_arch_->rfoot_fm->InitializeRampToMax(
      sp_->curr_time, ctrl_arch_->dcm_tm->getNormalForceRampUpTime());
  ctrl_arch_->lfoot_fm->InitializeRampToMax(
      sp_->curr_time, ctrl_arch_->dcm_tm->getNormalForceRampUpTime());

  // Initialize Task Heirarchy Ramp to Max
  ctrl_arch_->rfoot_pos_hm->InitializeRampToMax(
      sp_->curr_time, ctrl_arch_->dcm_tm->getNormalForceRampUpTime());
  ctrl_arch_->rfoot_ori_hm->InitializeRampToMax(
      sp_->curr_time, ctrl_arch_->dcm_tm->getNormalForceRampUpTime());
  ctrl_arch_->lfoot_pos_hm->InitializeRampToMax(
      sp_->curr_time, ctrl_arch_->dcm_tm->getNormalForceRampUpTime());
  ctrl_arch_->lfoot_ori_hm->InitializeRampToMax(
      sp_->curr_time, ctrl_arch_->dcm_tm->getNormalForceRampUpTime());

  // Check if it is the last footstep
  if (ctrl_arch_->dcm_tm->noRemainingSteps()) {
    end_time_ = ctrl_arch_->dcm_tm->getFinalContactTransferTime();
  } else {
    int transfer_type = DCM_TRANSFER_TYPES::MIDSTEP;
    end_time_ = ctrl_arch_->dcm_tm->getNormalForceRampUpTime();

    if (sp_->prev_state == draco_states::kBalance) {
      transfer_type = DCM_TRANSFER_TYPES::INITIAL;
      end_time_ = ctrl_arch_->dcm_tm->getInitialContactTransferTime() -
                  ctrl_arch_->dcm_tm->getNormalForceRampDownTime();

      // TODO : Replanning
      Eigen::Quaternion<double> torso_quat(
          robot_->get_link_iso("torso_link").linear());
      ctrl_arch_->dcm_tm->initialize(sp_->curr_time, transfer_type, torso_quat,
                                     sp_->dcm, sp_->dcm_vel);
      ctrl_arch_->dcm_tm->saveSolution(std::to_string(sp_->planning_id));
      sp_->planning_id += 1;
    }
  }
}

void ContactTransitionStart::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // Update max normal reaction forces
  ctrl_arch_->rfoot_fm->UpdateRampToMax(sp_->curr_time);
  ctrl_arch_->lfoot_fm->UpdateRampToMax(sp_->curr_time);

  // Update task hierarchy
  ctrl_arch_->rfoot_pos_hm->UpdateRampToMax(sp_->curr_time);
  ctrl_arch_->rfoot_ori_hm->UpdateRampToMax(sp_->curr_time);
  ctrl_arch_->lfoot_pos_hm->UpdateRampToMax(sp_->curr_time);
  ctrl_arch_->lfoot_ori_hm->UpdateRampToMax(sp_->curr_time);

  // Update Foot Task
  ctrl_arch_->rfoot_tm->useCurrent();
  ctrl_arch_->lfoot_tm->useCurrent();

  // Update floating base task
  ctrl_arch_->dcm_tm->updateDCMTasksDesired(sp_->curr_time);
}

void ContactTransitionStart::lastVisit() {}

bool ContactTransitionStart::endOfState() {

  if (state_machine_time_ >= end_time_) {
    return true;
  } else {
    return false;
  }
}

StateIdentifier ContactTransitionStart::getNextState() {
  if (ctrl_arch_->dcm_tm->noRemainingSteps()) {
    return draco_states::kBalance;
  } else {
    if (leg_side_ == EndEffector::LFoot) {
      sp_->stance_foot = "r_foot_contact";
      return draco_states::kLFootContactTransitionEnd;
    } else if (leg_side_ == EndEffector::RFoot) {
      sp_->stance_foot = "l_foot_contact";
      return draco_states::kRFootContactTransitionEnd;
    }
  }
}
