#pragma once

#include <map>

#include <Configuration.h>
#include <PnC/StateMachine.hpp>

#include <PnC/RobotSystem/RobotSystem.hpp>

#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>

// Generic Control Architecture Object
class ControlArchitecture {
 public:
  ControlArchitecture(RobotSystem* _robot) {
    DataManager::GetDataManager()->RegisterData(&state_, INT, "phase");
    robot_ = _robot;
  };
  virtual ~ControlArchitecture(){};

  virtual void ControlArchitectureInitialization() = 0;
  virtual void getCommand(void* _command){};

  int getState() { return state_; }
  int getPrevState() { return prev_state_; }
  RobotSystem* robot_;

  std::map<StateIdentifier, StateMachine*> state_machines_;

 protected:
  int state_;
  int prev_state_;
};
