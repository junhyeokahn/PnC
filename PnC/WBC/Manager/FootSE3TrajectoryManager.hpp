#pragma once

#include <PnC/Planner/Locomotion/DCMPlanner/DCMPlanner.hpp>
#include <PnC/Planner/Locomotion/DCMPlanner/Footstep.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/WBC/Contact.hpp>
#include <PnC/WBC/Task.hpp>
#include <Utils/Math/HermiteCurveVec.hpp>
#include <Utils/Math/HermiteQuaternionCurve.hpp>

// Object to manage common trajectory primitives
class FootSE3TrajectoryManager {
public:
  FootSE3TrajectoryManager(Task *_foot_pos_task, Task *_foot_ori_task,
                           RobotSystem *_robot);
  ~FootSE3TrajectoryManager();
  void paramInitialization(const YAML::Node &node);

  // Use current pose to set the task.
  void useCurrent();

  Task *foot_pos_task_;
  Task *foot_ori_task_;
  std::string link_idx_;

  Eigen::Vector3d foot_pos_des_;
  Eigen::Vector3d foot_vel_des_;
  Eigen::Vector3d foot_acc_des_;

  Eigen::Quaterniond foot_quat_des_;
  Eigen::VectorXd foot_ori_des_;
  Eigen::Vector3d foot_ang_vel_des_;
  Eigen::Vector3d foot_ang_acc_des_;

  // Updates the task desired values
  void updateDesired();

  // Initialize the swing foot trajectory
  void initializeSwingFootTrajectory(const double _start_time,
                                     const double _swing_duration,
                                     const Footstep &_landing_foot);
  // Computes the swing foot
  void computeSwingFoot(const double current_time);
  // computes the swing foot and updates the desired swing foot task
  void updateSwingFootDesired(const double current_time);

  // For TOWR+
  // void updateSwingFootDesired(const Eigen::Vector3d pos,
  // const Eigen::Vector3d vel,
  // const Eigen::Quaternion<double> quat,
  // const Eigen::Vector3d ang_vel);

  double swing_start_time_;
  double swing_duration_;
  double swing_height_;

  // Swing foot containers to create hermite curve
  Footstep swing_init_foot_;
  Footstep swing_midfoot_;
  Footstep swing_land_foot_;

  // Hermite Curve containers
  HermiteCurveVec pos_traj_init_to_mid_;
  HermiteCurveVec pos_traj_mid_to_end_;
  HermiteQuaternionCurve quat_hermite_curve_;

protected:
  void convertQuatDesToOriDes();
  RobotSystem *robot_;
};
