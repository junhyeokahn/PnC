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
                                     Eigen::VectorXd com_vel_des,
                                     const Eigen::VectorXd _end_foot_pos);
  // Computes the swing foot
  void computeSwingFoot(const double current_time);
  // computes the swing foot and updates the desired swing foot task
  void updateSwingFootDesired(const double current_time);

  double swing_start_time_;
  double swing_duration_;
  double swing_height_;

  // Hermite Curve containers
  HermiteCurveVec pos_traj_init_to_mid_;
  HermiteCurveVec pos_traj_mid_to_end_;
  HermiteQuaternionCurve quat_hermite_curve_;
 protected:

};
