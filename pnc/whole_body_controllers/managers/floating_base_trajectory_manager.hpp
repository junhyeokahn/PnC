#pragma once

#include "pnc/whole_body_controllers/basic_tasks.hpp"
#include "utils/interpolation.hpp"
#include "utils/util.hpp"

/// class FloatingBaseTrajectoryManager
class FloatingBaseTrajectoryManager {
public:
  /// \{ \name Constructor and Destructor
  FloatingBaseTrajectoryManager(Task *_com_task, Task *_base_ori_task,
                                RobotSystem *_robot);
  ~FloatingBaseTrajectoryManager(){};
  /// \}

  /// Initialize floating base interpolation to _target_com_pos and
  /// _target_base_quat
  void InitializeFloatingBaseInterpolationTrajectory(
      const double _start_time, const double _duration,
      const Eigen::Vector3d &_target_com_pos,
      const Eigen::Quaternion<double> &_target_base_quat);

  /// Initialize swaying trajectory
  void InitializeCoMSwayingTrajectory(double _start_time,
                                      const Eigen::Vector3d &_amp,
                                      const Eigen::Vector3d &_freq);

  /// Update floating base task commands.
  void UpdateFloatingBaseDesired(const double current_time);

private:
  RobotSystem *robot_;

  Eigen::Vector3d amp_;
  Eigen::Vector3d freq_;

  std::string base_id_;

  Task *com_task_;
  Task *base_ori_task_;

  bool b_swaying_;

  double start_time_;
  double duration_;

  Eigen::Vector3d ini_com_pos_;
  Eigen::VectorXd target_com_pos_;
  Eigen::Quaternion<double> ini_base_quat_;
  Eigen::Quaternion<double> target_base_quat_;
  Eigen::Vector3d exp_error_;
};
