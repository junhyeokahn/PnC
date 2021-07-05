#pragma once

#include "PnC/WBC/BasicTask.hpp"
#include "utils/interpolation.hpp"
#include "utils/util.hpp"

// Object to manage common trajectory primitives
class FloatingBaseTrajectoryManager {
public:
  FloatingBaseTrajectoryManager(Task *_com_task, Task *_base_ori_task,
                                RobotSystem *_robot);
  ~FloatingBaseTrajectoryManager(){};

  void InitializeFloatingBaseInterpolationTrajectory(
      const double _start_time, const double _duration,
      const Eigen::Vector3d &_target_com_pos,
      const Eigen::Quaternion<double> &_target_base_quat);
  void InitializeCoMSwayingTrajectory(double _start_time,
                                      const Eigen::Vector3d &_amp,
                                      const Eigen::Vector3d &_freq);
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
