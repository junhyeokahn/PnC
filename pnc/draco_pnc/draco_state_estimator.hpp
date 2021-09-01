#pragma once

#include <vector>

#include <Eigen/Dense>

#include "configuration.hpp"
#include "pnc/filters/digital_filters.hpp"

class DracoStateProvider;
class RobotSystem;
class DracoSensorData;

class DracoStateEstimator {
public:
  DracoStateEstimator(RobotSystem *robot);
  ~DracoStateEstimator();

  void initialize(DracoSensorData *);
  void update(DracoSensorData *);
  void update_debug(DracoSensorData *);

protected:
  DracoStateProvider *sp_;
  RobotSystem *robot_;

  Eigen::Isometry3d iso_base_joint_to_imu_;
  Eigen::Isometry3d iso_base_com_to_imu_;

  Eigen::Vector3d global_linear_offset_;
  Eigen::Vector3d prev_base_joint_pos_;
  Eigen::Vector3d prev_base_com_pos_;

  std::vector<SimpleMovingAverage> com_vel_filter_;
  std::vector<SimpleMovingAverage> imu_ang_vel_filter_;
  std::vector<SimpleMovingAverage> cam_filter_;

  bool b_first_visit_;

  void ComputeDCM();
};
