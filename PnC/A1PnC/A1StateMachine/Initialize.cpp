#include <PnC/A1PnC/A1CtrlArchitecture/A1CtrlArchitecture.hpp>
#include <PnC/A1PnC/A1StateMachine/Initialize.hpp>

Initialize::Initialize(const StateIdentifier state_identifier_in,
                       A1ControlArchitecture* _ctrl_arch, 
                       RobotSystem* _robot)
    : StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Initialize");

  // Set Pointer to Control Architecture
  ctrl_arch_ = ((A1ControlArchitecture*)_ctrl_arch);
  taf_container_ = ctrl_arch_->taf_container_;
  // Get State Provider
  sp_ = A1StateProvider::getStateProvider(robot_);
}

Initialize::~Initialize() {}

void Initialize::firstVisit() {
  std::cout << "[Initialize] Start" << std::endl;
  // Reset Flags
  ctrl_start_time_ = sp_->curr_time;
  ctrl_arch_->floating_base_lifting_up_manager_->initializeFloatingBaseTrajectory(
        0., end_time_, target_pos_)
}

void Initialize::_taskUpdate() {
  // =========================================================================
  // Foot, Floating Base
  // =========================================================================
  ctrl_arch_->floating_base_lifting_up_manager_->useCurrent();
  // ctrl_arch_->lfoot_trajectory_manager_->useCurrent();
}

void Initialize::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  _taskUpdate();
}

void Initialize::lastVisit() {}

bool DoubleSupportBalance::endOfState() {
  // Also check if footstep list is non-zero
  if (state_machine_time_ > end_time_) {
    return true;
  }
  return false;
}

StateIdentifier Initialize::getNextState() {
/*  int robot_side;
  // Check if there's a valid step
  if (ctrl_arch_->dcm_trajectory_manager_->nextStepRobotSide(robot_side)) {
    // Check which side is the next footstep
    if (robot_side == LEFT_ROBOT_SIDE) {
      return DRACO_STATES::LL_CONTACT_TRANSITION_START;
    } else {
      return DRACO_STATES::RL_CONTACT_TRANSITION_START;
    }
  }*/
}

void Initialize::initialization(const YAML::Node& node) {

  try{
    myUtils::readParameter(node, "target_pos_duration", end_time_);
    myUtils::readParameter(node, "com_pos_target", target_pos_);
  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0)
  }
}
