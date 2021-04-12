#include <PnC/A1PnC/A1CtrlArchitecture/A1CtrlArchitecture.hpp>
#include <PnC/A1PnC/A1StateMachine/ContactTransitionStart.hpp>

ContactTransitionStart::ContactTransitionStart(
    const StateIdentifier state_identifier_in, const int _front_leg_side,
    A1ControlArchitecture* _ctrl_arch, RobotSystem* _robot)
    : StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Contact Transition");

  // Set Pointer to Control Architecture
  ctrl_arch_ = ((A1ControlArchitecture*)_ctrl_arch);
  taf_container_ = ctrl_arch_->taf_container_;
  // Get State Provider
  sp_ = A1StateProvider::getStateProvider(robot_);

  // Set Leg Side
  front_leg_side_ = _front_leg_side;
}

ContactTransitionStart::~ContactTransitionStart() {}

void ContactTransitionStart::firstVisit() {
  // Manage task and contact list
  taf_container_->task_list_.clear();
  taf_container_->task_list_.push_back(taf_container_->com_task_);
  taf_container_->task_list_.push_back(taf_container_->base_ori_task);
  taf_container_->contact_list_.clear();
  taf_container_->contact_list_.push_back(taf_container_->flfoot_contact_);
  taf_container_->contact_list_.push_back(taf_container_->frfoot_contact_);
  taf_container_->contact_list_.push_back(taf_container_->rlfoot_contact_);
  taf_container_->contact_list_.push_back(taf_container_->rrfoot_contact_);
  // Set control Starting time
  if (state_identity_ == A1_STATES::FL_CONTACT_TRANSITION_START) {
    std::cout << "[Front Left Foot Contact Transition Start]" << std::endl;
  } else {
    std::cout << "[Front Right Contact Transition Start]" << std::endl;
  }
  ctrl_start_time_ = sp_->curr_time;

  // For all contact transitions, initially ramp up the reaction forces to max
  ctrl_arch_->frfoot_max_normal_force_manager_->initializeRampToMax(
      0.0, ramp_time_);
  ctrl_arch_->flfoot_max_normal_force_manager_->initializeRampToMax(
      0.0, ramp_time_);
  ctrl_arch_->rrfoot_max_normal_force_manager_->initializeRampToMax(
      0.0, ramp_time_);
  ctrl_arch_->rlfoot_max_normal_force_manager_->initializeRampToMax(
      0.0, ramp_time_);

  sp_->planning_id += 1;
  }
}

void ContactTransitionStart::_taskUpdate() {
  // =========================================================================
  // Floating Base
  // =========================================================================
  // ctrl_arch_->dcm_trajectory_manager_->updateDCMTasksDesired(sp_->curr_time);

}

void ContactTransitionStart::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // =========================================================================
  // Compute and update new maximum reaction forces
  // =========================================================================
  /*  ctrl_arch_->lfoot_front_max_normal_force_manager_->updateRampToMaxDesired(*/
  // state_machine_time_);
  // ctrl_arch_->lfoot_back_max_normal_force_manager_->updateRampToMaxDesired(
  // state_machine_time_);
  // ctrl_arch_->rfoot_front_max_normal_force_manager_->updateRampToMaxDesired(
  // state_machine_time_);
  // ctrl_arch_->rfoot_back_max_normal_force_manager_->updateRampToMaxDesired(
  /*state_machine_time_);*/
  ctrl_arch_->rfoot_max_normal_force_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->lfoot_max_normal_force_manager_->updateRampToMaxDesired(
      state_machine_time_);

  _taskUpdate();
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

  if (state_identity_ == A1_STATES::FL_CONTACT_TRANSITION_START) {
    sp_->front_stance_foot = A1BodyNode::FR_foot;
    sp_->rear_stance_foot = A1BodyNode::RL_foot;
    return A1_STATES::FL_CONTACT_TRANSITION_END;
  } else if (state_identity_ == A1_STATES::FR_CONTACT_TRANSITION_START) {
    sp_->front_stance_foot = A1BodyNode::FL_foot;
    sp_->front_stance_foot = A1BodyNode::RR_foot;
    return A1_STATES::FR_CONTACT_TRANSITION_END;
  }

}

void ContactTransitionStart::initialization(const YAML::Node& node) {
    myUtils::readParameter(node, "ramp_time", ramp_time_);
}