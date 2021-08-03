#pragma once

#include <Eigen/Dense>

#include "configuration.hpp"

class FixedDracoStateProvider;
class RobotSystem;
class FixedDracoSensorData;

class FixedDracoStateEstimator {
public:
  FixedDracoStateEstimator(RobotSystem *robot);
  ~FixedDracoStateEstimator();

  void initialize(FixedDracoSensorData *);
  void update(FixedDracoSensorData *);

protected:
  FixedDracoStateProvider *sp_;
  RobotSystem *robot_;

  Eigen::Isometry3d iso_base_joint_to_imu_;
  Eigen::Isometry3d iso_base_com_to_imu_;
};
