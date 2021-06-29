#pragma once

#include <Eigen/Dense>

#include "Configuration.hpp"

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

  void _update_dcm();
};
