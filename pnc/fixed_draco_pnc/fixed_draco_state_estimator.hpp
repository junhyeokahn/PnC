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

  Eigen::Quaternion<double> slack_quat_;
};
