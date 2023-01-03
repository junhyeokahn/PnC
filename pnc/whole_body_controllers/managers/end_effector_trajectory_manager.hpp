#pragma once

#include "pnc/planners/locomotion/dcm_planner/dcm_planner.hpp"
#include "pnc/planners/locomotion/dcm_planner/footstep.hpp"
#include "pnc/robot_system/robot_system.hpp"
#include "pnc/whole_body_controllers/contact.hpp"
#include "pnc/whole_body_controllers/task.hpp"
#include "utils/interpolation/interpolation.hpp"
#include "utils/util.hpp"

/// class EndEffectorTrajectoryManager
class EndEffectorTrajectoryManager {
public:
  /// \{ \name Constructor and Destructor
  EndEffectorTrajectoryManager(Task *_ee_pos_task, Task *_ee_ori_task,
                               RobotSystem *_robot);
  ~EndEffectorTrajectoryManager();
  /// \}

  /// Initialize interpolation trajectory
  void InitializeInterpolationTrajectory(
      const double &_start_time, const double &_duration,
      const Eigen::Vector3d &_target_pos,
      const Eigen::Quaternion<double> &_target_quat);

  /// Initialize swaying trajectory.
  void InitializeSwayingTrajectory(const double &_start_time,
                                   const Eigen::Vector3d &_amp,
                                   const Eigen::Vector3d &_freq);

  /// Update task desired using the planner initialized.
  void UpdateDesired(const double &_curr_time);

  /// Update task desired using the target isometry.
  void UpdateDesired(const Eigen::Isometry3d &target_ee_iso);

  /// Use current pose and vel to set the task to make zero acceleration.
  void UpdateZeroAccCmd();

private:
  RobotSystem *robot_;

  Eigen::Vector3d amp_;
  Eigen::Vector3d freq_;

  std::string link_idx_;

  bool b_swaying_;
  double start_time_;
  double duration_;

  Task *pos_task_;
  Task *ori_task_;

  Eigen::Vector3d ini_pos_;
  Eigen::Vector3d target_pos_;
  Eigen::Quaternion<double> ini_quat_;
  Eigen::Quaternion<double> target_quat_;
  Eigen::Vector3d exp_error_;
};
