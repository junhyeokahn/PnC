#pragma once

#include <Eigen/Dense>

#include "configuration.hpp"

class DracoStateProvider;
class RobotSystem;
class DracoSensorData;

class DracoStateEstimator {
public:
  DracoStateEstimator(RobotSystem *robot);
  ~DracoStateEstimator();

  void initialize(DracoSensorData *);
  void update(DracoSensorData *);

protected:
  DracoStateProvider *sp_;
  RobotSystem *robot_;

  Eigen::Isometry3d iso_base_joint_to_imu_;
  Eigen::Isometry3d iso_base_com_to_imu_;

  Eigen::Vector3d global_linear_offset_;
  Eigen::Vector3d prev_base_joint_pos_;
  Eigen::Vector3d prev_base_com_pos_;

  void _update_dcm();
};
