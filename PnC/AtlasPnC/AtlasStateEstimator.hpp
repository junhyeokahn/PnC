#pragma once

#include <Configuration.h>
#include <Eigen/Dense>

class AtlasStateProvider;
class RobotSystem;
class AtlasSensorData;

class AtlasStateEstimator {
public:
  AtlasStateEstimator(RobotSystem *robot);
  ~AtlasStateEstimator();

  void initialization(AtlasSensorData *);
  void update(AtlasSensorData *);

protected:
  AtlasStateProvider *sp_;
  RobotSystem *robot_;

  void _update_dcm();
};
