#pragma once

#include <PnC/Planner/Footstep.hpp>
#include <PnC/TrajectoryManager/TrajectoryManagerBase.hpp>
#include <PnC/WBC/BasicContactSpec.hpp>
#include <PnC/WBC/BasicTask.hpp>
#include <Utils/Math/hermite_curve_vec.hpp>
#include <Utils/Math/hermite_quaternion_curve.hpp>

// Object to manage common trajectory primitives
class PointFootTrajectoryManager : public TrajectoryManagerBase {
 public:
  PointFootTrajectoryManager(Task* _foot_pos_task, RobotSystem* _robot);
  ~PointFootTrajectoryManager();
  void paramInitialization(const YAML::Node& node);

  // Use current pose to set the task.
  void useCurrent();

  Task* foot_pos_task_;
  int link_idx_;

  Eigen::Vector3d foot_pos_des_;
  Eigen::Vector3d foot_vel_des_;
  Eigen::Vector3d foot_acc_des_;

  // Updates the task desired values
  void updateDesired();

  // Initialize the swing foot trajectory
  void initializeSwingFootTrajectory(const double _start_time,
                                     const double _swing_duration,
                                     const Footstep& _landing_foot);
  // Computes the swing foot
  void computeSwingFoot(const double current_time);
  // computes the swing foot and updates the desired swing foot task
  void updateSwingFootDesired(const double current_time);

 protected:
  double swing_height_;
};