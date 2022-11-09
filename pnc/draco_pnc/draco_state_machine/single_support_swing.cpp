#include "pnc/draco_pnc/draco_state_machine/single_support_swing.hpp"

SingleSupportSwing::SingleSupportSwing(const StateIdentifier _state_identifier,
                                       DracoControlArchitecture *_ctrl_arch,
                                       int _leg_side, RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  util::PrettyConstructor(2, "SingleSupportSwing");

  ctrl_arch_ = _ctrl_arch;
  leg_side_ = _leg_side;
  sp_ = DracoStateProvider::getStateProvider();
}

SingleSupportSwing::~SingleSupportSwing() {}

void SingleSupportSwing::firstVisit() {
  if (leg_side_ == EndEffector::RFoot) {
    std::cout << "draco_states::kRFootSwing" << std::endl;
  } else {
    std::cout << "draco_states::kLFootSwing" << std::endl;
  }

  ctrl_start_time_ = sp_->curr_time;
  end_time_ = ctrl_arch_->dcm_tm->getSwingTime();

  int footstep_idx = ctrl_arch_->dcm_tm->current_footstep_idx;

  if (leg_side_ == EndEffector::RFoot) {
    // rfoot swing
    ctrl_arch_->rfoot_tm->InitializeSwingTrajectory(
        sp_->curr_time, end_time_, sp_->nominal_rfoot_iso,
        ctrl_arch_->dcm_tm->footstep_list[footstep_idx]);
    sp_->b_rf_contact = false;
    sp_->b_lf_contact = true;
  } else if (leg_side_ == EndEffector::LFoot) {
    // lfoot swing
    ctrl_arch_->lfoot_tm->InitializeSwingTrajectory(
        sp_->curr_time, end_time_, sp_->nominal_lfoot_iso,
        ctrl_arch_->dcm_tm->footstep_list[footstep_idx]);
    sp_->b_rf_contact = true;
    sp_->b_lf_contact = false;
  } else {
    assert(false);
  }
}

void SingleSupportSwing::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // Update Foot Task
  if (leg_side_ == EndEffector::LFoot) {
    ctrl_arch_->lfoot_tm->UpdateDesired(sp_->curr_time);
    ctrl_arch_->rfoot_tm->useNominalPoseCmd(sp_->nominal_rfoot_iso);
//    ctrl_arch_->rfoot_tm->UpdateZeroAccCmd();
  } else {
    ctrl_arch_->rfoot_tm->UpdateDesired(sp_->curr_time);
    ctrl_arch_->lfoot_tm->useNominalPoseCmd(sp_->nominal_lfoot_iso);
//    ctrl_arch_->lfoot_tm->UpdateZeroAccCmd();
  }

  // Update floating base task
  ctrl_arch_->dcm_tm->updateDCMTasksDesired(sp_->curr_time);
}

void SingleSupportSwing::lastVisit() {
  ctrl_arch_->dcm_tm->incrementStepIndex();

  sp_->nominal_lfoot_iso.translation() = ctrl_arch_->lfoot_tm->GetDesiredPos();
  sp_->nominal_lfoot_iso.linear() = ctrl_arch_->lfoot_tm->GetDesiredOri();
  sp_->nominal_rfoot_iso.translation() = ctrl_arch_->rfoot_tm->GetDesiredPos();
  sp_->nominal_rfoot_iso.linear() = ctrl_arch_->rfoot_tm->GetDesiredOri();

  sp_->b_rf_contact = true;
  sp_->b_lf_contact = true;
}

bool SingleSupportSwing::endOfState() {

  if (b_early_termination) {
    if (state_machine_time_ >= 0.5 * end_time_) {
      // use foot position
      if (leg_side_ == EndEffector::RFoot) {
        double rfoot_height =
            robot_->get_link_iso("r_foot_contact").translation()[2];
        if (rfoot_height <= foot_height_threshold) {
          std::cout << "[Early Termination]: Rfoot contact happen at "
                    << state_machine_time_ << " / " << end_time_ << std::endl;
          return true;
        }
      } else if (leg_side_ == EndEffector::LFoot) {
        double lfoot_height =
            robot_->get_link_iso("l_foot_contact").translation()[2];
        if (lfoot_height <= foot_height_threshold) {
          std::cout << "[Early Termination]: Lfoot contact happen at "
                    << state_machine_time_ << " / " << end_time_ << std::endl;
          return true;
        }
      } else {
        assert(false);
      }
      // TODO : use force sensor
    }
  }

  if (state_machine_time_ >= end_time_) {
    return true;
  } else {
    return false;
  }

  // if (state_machine_time_ >= end_time_) {
  // return true;
  //} else {
  // if (state_machine_time_ >= 0.5 * end_time_) {
  // if (leg_side_ == EndEffector::LFoot) {
  // if (sp_->b_lf_contact) {
  // printf("Early left foot contact at %f/%f\n", state_machine_time_,
  // end_time_);
  // return true;
  //}
  //} else {
  // if (sp_->b_rf_contact) {
  // printf("Early right foot contact at %f/%f\n", state_machine_time_,
  // end_time_);
  // return true;
  //}
  //}
  //}
  // return false;
  //}
}

StateIdentifier SingleSupportSwing::getNextState() {
  int next_footstep_robot_side;
  if (ctrl_arch_->dcm_tm->nextStepRobotSide(next_footstep_robot_side)) {
    if (next_footstep_robot_side == EndEffector::LFoot) {
      return draco_states::kLFootContactTransitionStart;
    } else {
      return draco_states::kRFootContactTransitionStart;
    }
  } else {
    if (leg_side_ == EndEffector::LFoot) {
      return draco_states::kLFootContactTransitionStart;
    } else {
      return draco_states::kRFootContactTransitionStart;
    }
  }
}
