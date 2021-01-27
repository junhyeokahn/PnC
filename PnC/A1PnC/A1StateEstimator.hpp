#pragma once

#include <Configuration.h>
#include <Eigen/Dense>

class A1StateProvider;
class RobotSystem;
class A1SensorData;

class A1StateEstimator {
 public:
  A1StateEstimator(RobotSystem* robot);
  ~A1StateEstimator();

  void Initialization(A1SensorData*);
  void Update(A1SensorData*);

 protected:
  A1StateProvider* sp_;
  RobotSystem* robot_;

  Eigen::VectorXd curr_config_;
  Eigen::VectorXd curr_qdot_;

  void _JointUpdate(A1SensorData* data);
  void _ConfigurationAndModelUpdate();
  void _FootContactUpdate(A1SensorData* data);
};
