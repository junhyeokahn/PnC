#include <pnc/atlas_pnc/atlas_state_machine/contact_transition_start.hpp>

ContactTransitionStart::ContactTransitionStart(
    const StateIdentifier _state_identifier,
    AtlasControlArchitecture *_ctrl_arch, int _leg_side, RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  util::PrettyConstructor(2, "ContactTransitionStart");

  atlas_ctrl_arch_ = _ctrl_arch;
  leg_side_ = _leg_side;
  sp_ = AtlasStateProvider::getStateProvider(_robot);
}

ContactTransitionStart::~ContactTransitionStart() {}

void ContactTransitionStart::firstVisit() {
  if (leg_side_ == EndEffector::RFoot) {
    std::cout << "AtlasState::RFootContactTransitionStart" << std::endl;
  } else {
    std::cout << "AtlasState::LFootContactTransitionStart" << std::endl;
  }

  ctrl_start_time_ = sp_->curr_time;

  // Initialize Reaction Force Ramp to Max
  atlas_ctrl_arch_->rfoot_fm->InitializeRampToMax(
      sp_->curr_time, atlas_ctrl_arch_->dcm_tm->getNormalForceRampUpTime());
  atlas_ctrl_arch_->lfoot_fm->InitializeRampToMax(
      sp_->curr_time, atlas_ctrl_arch_->dcm_tm->getNormalForceRampUpTime());

  // Initialize Task Heirarchy Ramp to Max
  atlas_ctrl_arch_->rfoot_pos_hm->InitializeRampToMax(
      sp_->curr_time, atlas_ctrl_arch_->dcm_tm->getNormalForceRampUpTime());
  atlas_ctrl_arch_->rfoot_ori_hm->InitializeRampToMax(
      sp_->curr_time, atlas_ctrl_arch_->dcm_tm->getNormalForceRampUpTime());
  atlas_ctrl_arch_->lfoot_pos_hm->InitializeRampToMax(
      sp_->curr_time, atlas_ctrl_arch_->dcm_tm->getNormalForceRampUpTime());
  atlas_ctrl_arch_->lfoot_ori_hm->InitializeRampToMax(
      sp_->curr_time, atlas_ctrl_arch_->dcm_tm->getNormalForceRampUpTime());

  // Check if it is the last footstep
  if (atlas_ctrl_arch_->dcm_tm->noRemainingSteps()) {
    end_time_ = atlas_ctrl_arch_->dcm_tm->getFinalContactTransferTime();
  } else {
    int transfer_type = DCM_TRANSFER_TYPES::MIDSTEP;
    end_time_ = atlas_ctrl_arch_->dcm_tm->getNormalForceRampUpTime();

    if (sp_->prev_state == AtlasStates::Balance) {
      transfer_type = DCM_TRANSFER_TYPES::INITIAL;
      end_time_ = atlas_ctrl_arch_->dcm_tm->getInitialContactTransferTime() -
                  atlas_ctrl_arch_->dcm_tm->getNormalForceRampDownTime();

      // TODO : Replanning
      Eigen::Quaternion<double> pelvis_quat(
          robot_->get_link_iso("pelvis").linear());
      atlas_ctrl_arch_->dcm_tm->initialize(sp_->curr_time, transfer_type,
                                           pelvis_quat, sp_->dcm, sp_->dcm_vel);
      atlas_ctrl_arch_->dcm_tm->saveSolution(std::to_string(sp_->planning_id));
      sp_->planning_id += 1;
    }
  }
}

void ContactTransitionStart::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // Update max normal reaction forces
  atlas_ctrl_arch_->rfoot_fm->UpdateRampToMax(sp_->curr_time);
  atlas_ctrl_arch_->lfoot_fm->UpdateRampToMax(sp_->curr_time);

  // Update task hierarchy
  atlas_ctrl_arch_->rfoot_pos_hm->UpdateRampToMax(sp_->curr_time);
  atlas_ctrl_arch_->rfoot_ori_hm->UpdateRampToMax(sp_->curr_time);
  atlas_ctrl_arch_->lfoot_pos_hm->UpdateRampToMax(sp_->curr_time);
  atlas_ctrl_arch_->lfoot_ori_hm->UpdateRampToMax(sp_->curr_time);

  // Update Foot Task
  atlas_ctrl_arch_->rfoot_tm->useCurrent();
  atlas_ctrl_arch_->lfoot_tm->useCurrent();

  // Update floating base task
  atlas_ctrl_arch_->dcm_tm->updateDCMTasksDesired(sp_->curr_time);
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
  if (atlas_ctrl_arch_->dcm_tm->noRemainingSteps()) {
    return AtlasStates::Balance;
  } else {
    if (leg_side_ == EndEffector::LFoot) {
      return AtlasStates::LFootContactTransitionEnd;
    } else if (leg_side_ == EndEffector::RFoot) {
      return AtlasStates::RFootContactTransitionEnd;
    }
  }
}
