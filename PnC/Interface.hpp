#pragma once

#include <Eigen/Dense>

#include <PnC/InterruptLogic.hpp>
#include <Utils/IO/DataManager.hpp>

class ControlArchitecture;
class RobotSystem;

class Interface {
protected:
  ControlArchitecture *control_architecture_;
  RobotSystem *robot_;
  int count_;
  double running_time_;

public:
  Interface() {
    count_ = 0;
    running_time_ = 0.;
    DataManager::GetDataManager()->RegisterData(&running_time_, DOUBLE, "time");
  }
  InterruptLogic *interrupt;
  virtual ~Interface(){};

  // Get Command through Test
  virtual void getCommand(void *_sensor_data, void *_command_data) = 0;
};
