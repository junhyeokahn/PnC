#pragma once

#include <map>

#include <configuration.hpp>
#include <pnc/state_machine.hpp>

#include <pnc/robot_system/robot_system.hpp>

#include <utils/util.hpp>

// Generic Control Architecture Object
class ControlArchitecture {
public:
  ControlArchitecture(RobotSystem *_robot) { robot_ = _robot; };
  virtual ~ControlArchitecture(){};

  virtual void getCommand(void *_command){};

  std::map<StateIdentifier, StateMachine *> state_machines;
  int state;
  int prev_state;

protected:
  RobotSystem *robot_;

  bool b_state_first_visit_ = true;
};
