#include <PnC/TrajectoryManager/MaxNormalForceTrajectoryManager.hpp>

MaxNormalForceTrajectoryManager::MaxNormalForceTrajectoryManager(
    ContactSpec* _contact, RobotSystem* _robot)
    : TrajectoryManagerBase(_robot) {
  myUtils::pretty_constructor(2, "TrajectoryManager: Max Normal Force");
  contact_ = _contact;
  nominal_max_normal_force_z_ = 1500;                  // Default
  starting_max_normal_force_z_ = 0.0;                  // Default
  current_max_normal_force_z_ = _contact->getMaxFz();  // Default
  local_max_normal_force_z_ = 0.0;
  nominal_ramp_duration_ = 1.0;  // seconds
}

MaxNormalForceTrajectoryManager::~MaxNormalForceTrajectoryManager() {}

void MaxNormalForceTrajectoryManager::setMaxFz(const double _fr_max) {
  nominal_max_normal_force_z_ = _fr_max;
}

void MaxNormalForceTrajectoryManager::paramInitialization(
    const YAML::Node& node) {}

void MaxNormalForceTrajectoryManager::initializeRampToZero(
    const double start_time, const double nominal_ramp_duration) {
  // Initialize start times and starting max value
  ramp_start_time_ = start_time;
  starting_max_normal_force_z_ = current_max_normal_force_z_;
  nominal_ramp_duration_ = nominal_ramp_duration;
  // Initialize ramp speed
  ramp_down_speed_ = -nominal_max_normal_force_z_ / nominal_ramp_duration_;
}

void MaxNormalForceTrajectoryManager::initializeRampToMax(
    const double start_time, const double nominal_ramp_duration) {
  // Initialize start times and starting max value
  ramp_start_time_ = start_time;
  starting_max_normal_force_z_ = current_max_normal_force_z_;
  nominal_ramp_duration_ = nominal_ramp_duration;
  // Initialize ramp speed
  ramp_up_speed_ = nominal_max_normal_force_z_ / nominal_ramp_duration_;
}

void MaxNormalForceTrajectoryManager::computeRampToZero(
    const double current_time) {
  double t_current = myUtils::CropValue(current_time, ramp_start_time_,
                                        nominal_ramp_duration_);
  local_max_normal_force_z_ =
      ramp_down_speed_ * (t_current - ramp_start_time_) +
      starting_max_normal_force_z_;
  current_max_normal_force_z_ = myUtils::CropValue(
      local_max_normal_force_z_, 0.0, nominal_max_normal_force_z_);
}

void MaxNormalForceTrajectoryManager::computeRampToMax(
    const double current_time) {
  double t_current = myUtils::CropValue(current_time, ramp_start_time_,
                                        nominal_ramp_duration_);
  local_max_normal_force_z_ = ramp_up_speed_ * (t_current - ramp_start_time_) +
                              starting_max_normal_force_z_;
  current_max_normal_force_z_ = myUtils::CropValue(
      local_max_normal_force_z_, 0.0, nominal_max_normal_force_z_);
}

void MaxNormalForceTrajectoryManager::updateRampToZeroDesired(
    const double current_time) {
  computeRampToZero(current_time);
  updateMaxNormalForce();
}
void MaxNormalForceTrajectoryManager::updateRampToMaxDesired(
    const double current_time) {
  computeRampToMax(current_time);
  updateMaxNormalForce();
}

void MaxNormalForceTrajectoryManager::updateMaxNormalForce() {
  contact_->setMaxFz(current_max_normal_force_z_);
}
