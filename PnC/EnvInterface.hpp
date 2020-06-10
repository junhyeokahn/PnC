#pragma once

#include <Eigen/Dense>

#include <PnC/InterruptLogic.hpp>

class ControlArchitecture;
class RobotSystem;

class EnvInterface {
 protected:
  ControlArchitecture* control_architecture_;
  RobotSystem* robot_;
  InterruptLogic* interrupt_;
  int count_;
  double running_time_;

 public:
  EnvInterface() {
    count_ = 0;
    running_time_ = 0.;
  }
  virtual ~EnvInterface(){};

  // Get Command through Test
  virtual void getCommand(void* _sensor_data, void* _command_data) = 0;
};
