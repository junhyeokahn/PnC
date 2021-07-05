#pragma once

#include <map>

#include <Configuration.hpp>
#include <PnC/StateMachine.hpp>

#include <PnC/RobotSystem/RobotSystem.hpp>

//#include <Utils/IO/DataManager.hpp>
#include <utils/util.hpp>

// Generic Control Architecture Object
class ControlArchitecture {
public:
  ControlArchitecture(RobotSystem *_robot) {
    // DataManager::GetDataManager()->RegisterData(&state, INT, "phase");
    robot_ = _robot;
  };
  virtual ~ControlArchitecture(){};

  virtual void getCommand(void *_command){};

  std::map<StateIdentifier, StateMachine *> state_machines;
  int state;
  int prev_state;

protected:
  RobotSystem *robot_;

  bool b_state_first_visit_ = true;
};
