#pragma once

#include <Eigen/Dense>

#include <pnc/interrupt_logic.hpp>

class ControlArchitecture;
class RobotSystem;

/// class Interface
class Interface {

public:
  /// \{ \name Constructor and Destructor
  Interface() {
    count_ = 0;
    running_time_ = 0.;
  }

  virtual ~Interface(){};
  /// \}

  /// Receive sensor data from robot and returns commands. This method will
  /// be called at every servo loop in physics simulators or real hardware
  /// experiments.
  virtual void getCommand(void *_sensor_data, void *_command_data) = 0;

  InterruptLogic *interrupt;

protected:
  ControlArchitecture *control_architecture_;

  RobotSystem *robot_;

  /// Internal loop count
  int count_;

  /// Internal clock
  double running_time_;
};
