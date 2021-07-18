#pragma once

#include <Eigen/Dense>

#include <pnc/interrupt_logic.hpp>

class ControlArchitecture;
class RobotSystem;

/// class Interface
class Interface {
protected:
  ControlArchitecture *control_architecture_;
  RobotSystem *robot_;
  /// Internal count for control time
  int count_;
  /// Internal control time
  double running_time_;

public:
  /// \{ \name CConstructor and Destructor
  Interface() {
    count_ = 0;
    running_time_ = 0.;
  }
  virtual ~Interface(){};
  /// \}

  /// InterruptLogic
  InterruptLogic *interrupt;

  /// Receives sensor data from robot and returns commands
  virtual void getCommand(void *_sensor_data, void *_command_data) = 0;
};
