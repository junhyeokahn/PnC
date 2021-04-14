#include <PnC/A1PnC/A1CtrlArchitecture/A1CtrlArchitecture.hpp>
#include <PnC/A1PnC/A1StateMachine/ContactTransitionEnd.hpp>

ContactTransitionEnd::ContactTransitionEnd(
    const StateIdentifier state_identifier_in, const int _leg_side,
    A1ControlArchitecture* _ctrl_arch, RobotSystem* _robot)
    : StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Contact Transition End");

  // Set Pointer to Control Architecture
  ctrl_arch_ = ((A1ControlArchitecture*)_ctrl_arch);
  taf_container_ = ctrl_arch_->taf_container_;
  // Get State Provider
  sp_ = A1StateProvider::getStateProvider(robot_);

  // Set Leg Side
  leg_side_ = _leg_side;
}

ContactTransitionEnd::~ContactTransitionEnd() {}

void ContactTransitionEnd::firstVisit() {
  // Coming from ContactTransitionStart, do not need to alter task/contact lists
  // Set control Starting time
  if (state_identity_ == A1_STATES::FR_CONTACT_TRANSITION_END) {
    std::cout << "[Front Right Foot Contact Transition End]" << std::endl;
  } else {
    std::cout << "[Front Left Foot Contact Transition End]" << std::endl;
  }
  ctrl_start_time_ = sp_->curr_time;

  // Ramp Down Reaction force for the upcoming swing foot
  if (state_identity_ == A1_STATES::FL_CONTACT_TRANSITION_END) {
    ctrl_arch_->flfoot_max_normal_force_manager_->initializeRampToZero(
        0.0, ramp_time_);
    ctrl_arch_->rrfoot_max_normal_force_manager_->initializeRampToZero(
        0.0, ramp_time_);

   } else {
    ctrl_arch_->frfoot_max_normal_force_manager_->initializeRampToZero(
        0.0, ramp_time_);
    ctrl_arch_->rlfoot_max_normal_force_manager_->initializeRampToZero(
        0.0, ramp_time_);
  }
}

void ContactTransitionEnd::_taskUpdate() {
  // =========================================================================
  // Floating Base
  // =========================================================================
  ctrl_arch_->floating_base_lifting_up_manager_->updateFloatingBaseWalkingDesired(
                        sp_->com_pos,
                        sp_->x_y_yaw_vel_des);

}

void ContactTransitionEnd::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // =========================================================================
  // - Compute and update new maximum reaction forces
  // =========================================================================
  if (state_identity_ == A1_STATES::FL_CONTACT_TRANSITION_END) {
    ctrl_arch_->flfoot_max_normal_force_manager_->updateRampToZeroDesired(
        state_machine_time_);
    ctrl_arch_->rrfoot_max_normal_force_manager_->updateRampToZeroDesired(
        state_machine_time_);
  } else {
    ctrl_arch_->frfoot_max_normal_force_manager_->updateRampToZeroDesired(
        state_machine_time_);
    ctrl_arch_->rlfoot_max_normal_force_manager_->updateRampToZeroDesired(
        state_machine_time_);
  }

  _taskUpdate();
}

void ContactTransitionEnd::lastVisit() {}

bool ContactTransitionEnd::endOfState() {
  // if time exceeds transition time, switch state
  if (state_machine_time_ >= end_time_) {
    std::cout << "End of Contact Transition End Reachedf" << std::endl;
    return true;
  } else {
    return false;
  }
}

StateIdentifier ContactTransitionEnd::getNextState() {
  if (state_identity_ == A1_STATES::FL_CONTACT_TRANSITION_END) {
    // Switch to left leg swing
    return A1_STATES::FL_SWING;
  } else {
    // Switch to right leg swing
    return A1_STATES::FR_SWING;
  }
}

void ContactTransitionEnd::initialization(const YAML::Node& node) {
  myUtils::readParameter(node, "ramp_time", ramp_time_);
}
