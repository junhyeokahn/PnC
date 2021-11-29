#pragma once

#include "pnc/whole_body_controllers/basic_tasks.hpp"
#include "utils/interpolation.hpp"
#include "utils/util.hpp"

/// class FloatingBaseTrajectoryManager
class FloatingBaseTrajectoryManager {
public:
  /// \{ \name Constructor and Destructor
  FloatingBaseTrajectoryManager(Task *_com_task, Task *_base_ori_task,
                                RobotSystem *_robot,
                                const bool b_use_base_height = false);
  ~FloatingBaseTrajectoryManager(){};
  /// \}

  /// Initialize floating base interpolation to _target_com_pos and
  /// _target_base_quat
  //
  /// \param[in] _start_time Time at initialization.
  /// \param[in] _duration Duration for the trajectory.
  /// \param[in] _target_com_pos Target com pos in world frame.
  /// \param[in] _target_base_quat Target base quat in world frame.
  void InitializeInterpolationTrajectory(
      const double _start_time, const double _duration,
      const Eigen::Vector3d &_target_com_pos,
      const Eigen::Quaternion<double> &_target_base_quat);

  /// Initialize swaying trajectory
  //
  /// \param[in] _start_time Time at initialization
  /// \param[in] _local_amp Swaying amplitude in local contact frame.
  /// \param[in] _local_freq Swaying frequency in local contact frame.
  /// \param[in] _rot_world_local Rotation matrix of local contact frame w.r.t.
  /// world frame.
  void InitializeSwayingTrajectory(double _start_time,
                                   const Eigen::Vector3d &_local_amp,
                                   const Eigen::Vector3d &_local_freq,
                                   const Eigen::Matrix3d &_rot_world_local);

  /// Update floating base task commands.
  void UpdateDesired(const double current_time);

private:
  RobotSystem *robot_;

  Eigen::Vector3d local_amp_;
  Eigen::Vector3d local_freq_;

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

  Eigen::Matrix3d rot_world_local_;

  bool b_use_base_height_;
};
