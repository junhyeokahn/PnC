#include <PnC/DracoPnC/DracoCtrlArchitecture/DracoCtrlArchitecture.hpp>
#include <PnC/DracoPnC/DracoStateMachine/Initialize.hpp>

Initialize::Initialize(const StateIdentifier state_identifier_in,
                       DracoControlArchitecture* _ctrl_arch,
                       RobotSystem* _robot)
    : StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Initialize");

  // Set Pointer to Control Architecture
  ctrl_arch_ = ((DracoControlArchitecture*)_ctrl_arch);
  taf_container_ = ctrl_arch_->taf_container_;
  // Get State Provider
  sp_ = DracoStateProvider::getStateProvider(robot_);
}

Initialize::~Initialize() {}

void Initialize::firstVisit() {
  std::cout << "[Initialize]" << std::endl;

  ctrl_start_time_ = sp_->curr_time;
  ctrl_arch_->joint_trajectory_manager_->initializeJointTrajectory(
      0., end_time_, target_pos_);
}

void Initialize::_taskUpdate() {
  // =========================================================================
  // Joint
  // =========================================================================
  ctrl_arch_->joint_trajectory_manager_->updateJointDesired(
      state_machine_time_);
}

void Initialize::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  _taskUpdate();
}

void Initialize::lastVisit() {}

bool Initialize::endOfState() {
  if (state_machine_time_ > end_time_) {
    return true;
  }
  return false;
}

StateIdentifier Initialize::getNextState() { return DRACO_STATES::STAND; }

void Initialize::initialization(const YAML::Node& node) {
  try {
    myUtils::readParameter(node, "target_pos_duration", end_time_);
    myUtils::readParameter(node, "smoothing_duration", smoothing_dur_);
    myUtils::readParameter(node, "target_pos", target_pos_);
  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }
}
