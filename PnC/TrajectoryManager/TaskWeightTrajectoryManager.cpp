#include <PnC/TrajectoryManager/TaskWeightTrajectoryManager.hpp>

TaskWeightTrajectoryManager::TaskWeightTrajectoryManager(Task* _task,
                                                         RobotSystem* _robot)
    : TrajectoryManagerBase(_robot) {
  myUtils::pretty_constructor(2, "TrajectoryManager: Task Gain Schedule");
  task_ = _task;
  nominal_w_max_ = 40;                       // Default
  nominal_w_min_ = 20;                       // Default
  starting_w_ = nominal_w_max_;              // Default
  current_w_ = task_->getHierarchyWeight();  // Default
  local_w_ = 0.0;
  nominal_ramp_duration_ = 1.0;  // seconds
}

TaskWeightTrajectoryManager::~TaskWeightTrajectoryManager() {}

void TaskWeightTrajectoryManager::setMaxGain(const double _w_max) {
  nominal_w_max_ = _w_max;
}

void TaskWeightTrajectoryManager::setMinGain(const double _w_min) {
  nominal_w_min_ = _w_min;
}

void TaskWeightTrajectoryManager::paramInitialization(const YAML::Node& node) {}

void TaskWeightTrajectoryManager::initializeRampToMin(
    const double start_time, const double nominal_ramp_duration) {
  // Initialize start times and starting max value
  ramp_start_time_ = start_time;
  starting_w_ = current_w_;
  nominal_ramp_duration_ = nominal_ramp_duration;
  // Initialize ramp speed
  ramp_down_speed_ = -nominal_w_max_ / nominal_ramp_duration_;
}

void TaskWeightTrajectoryManager::initializeRampToMax(
    const double start_time, const double nominal_ramp_duration) {
  // Initialize start times and starting max value
  ramp_start_time_ = start_time;
  starting_w_ = current_w_;
  nominal_ramp_duration_ = nominal_ramp_duration;
  // Initialize ramp speed
  ramp_up_speed_ = nominal_w_max_ / nominal_ramp_duration_;
}

void TaskWeightTrajectoryManager::computeRampToMin(const double current_time) {
  double t_current = myUtils::CropValue(current_time, ramp_start_time_,
                                        nominal_ramp_duration_);
  local_w_ = ramp_down_speed_ * (t_current - ramp_start_time_) + starting_w_;
  current_w_ = myUtils::CropValue(local_w_, nominal_w_min_, nominal_w_max_);
}

void TaskWeightTrajectoryManager::computeRampToMax(const double current_time) {
  double t_current = myUtils::CropValue(current_time, ramp_start_time_,
                                        nominal_ramp_duration_);
  local_w_ = ramp_up_speed_ * (t_current - ramp_start_time_) + starting_w_;
  current_w_ = myUtils::CropValue(local_w_, nominal_w_min_, nominal_w_max_);
}

void TaskWeightTrajectoryManager::updateRampToMinDesired(
    const double current_time) {
  computeRampToMin(current_time);
  updateTaskWeight();
}
void TaskWeightTrajectoryManager::updateRampToMaxDesired(
    const double current_time) {
  computeRampToMax(current_time);
  updateTaskWeight();
}

void TaskWeightTrajectoryManager::updateTaskWeight() {
  task_->setHierarchyWeight(current_w_);
}
