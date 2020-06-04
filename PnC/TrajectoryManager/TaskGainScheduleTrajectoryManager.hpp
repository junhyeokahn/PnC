#pragma once

#include <PnC/TrajectoryManager/TrajectoryManagerBase.hpp>
#include <PnC/WBC/Task.hpp>
#include <Utils/Math/MathUtilities.hpp>

// Object to manage the hierarchy values o

// Call in the following order
// 1) initializeRampToMin/Max in the initialization step with the current time
// and nominal ramp duration
// 2) computeRampToMin() in the update step with the current time
// 3) updateTaskHierarchy() in the update step after computing

class TaskGainScheduleTrajectoryManager : public TrajectoryManagerBase {
 public:
  TaskGainScheduleTrajectoryManager(Task* _task, RobotSystem* _robot);
  ~TaskGainScheduleTrajectoryManager();
  void paramInitialization(const YAML::Node& node);

  Task* task_;

  double nominal_w_max_;
  double nominal_w_min_;

  // Initializes the ramp speed needed to bring the current max nominal force to
  // zero or max.
  // - Stores the starting time and value of the current nominal force
  // ramp_down_speed = -nominal_w_max_ / nominal_ramp_duration
  // ramp_up_speed = nominal_w_max_ / nominal_ramp_duration
  void initializeRampToMin(const double start_time,
                           const double nominal_ramp_duration);
  void initializeRampToMax(const double start_time,
                           const double nominal_ramp_duration);

  double ramp_start_time_;  // time at which ramping starts
  double starting_w_;       // starting normal force value

  // Updates the current max normal force given the current time and temporal
  // parameters
  void computeRampToMin(const double current_time);
  void computeRampToMax(const double current_time);

  double current_w_;  // current task hierarchy weight

  // computes the new normal force and updates the maximum force normal force
  void updateRampToMinDesired(const double current_time);
  void updateRampToMaxDesired(const double current_time);

  // Update the maximum normal force
  void updateTaskHierarchy();

 protected:
  double local_w_;
  double nominal_ramp_duration_;
  double ramp_up_speed_ = 1;
  double ramp_down_speed_ = 1;
};
