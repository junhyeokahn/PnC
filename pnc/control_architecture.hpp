#pragma once

#include <map>

#include "configuration.hpp"
#include "pnc/robot_system/robot_system.hpp"
#include "pnc/state_machine.hpp"
#include "utils/util.hpp"

/// class ControlArchitecture
class ControlArchitecture {
public:
  /// { \name Constructor and Destructor
  ControlArchitecture(RobotSystem *_robot) { robot_ = _robot; };
  virtual ~ControlArchitecture(){};
  /// \}

  /// Compute commands.
  virtual void getCommand(void *_command){};

  /// Map of StateIdentifier and StateMachine
  std::map<StateIdentifier, StateMachine *> state_machines;

  /// Placeholder for the state at current time step
  int state;

  /// Placeholder for the state at previous time step
  int prev_state;

protected:
  RobotSystem *robot_;

  bool b_state_first_visit_ = true;
};
