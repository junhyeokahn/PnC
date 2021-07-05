#pragma once

#include <pnc/robot_system/robot_system.hpp>
#include <pnc/whole_body_controllers/contact.hpp>
#include <pnc/whole_body_controllers/task.hpp>
#include <utils/util.hpp>

typedef int StateIdentifier;

class StateMachine {
public:
  StateMachine(const StateIdentifier state_identifier_in, RobotSystem *_robot) {
    robot_ = _robot;
    state_machine_time_ = 0.;
    state_identity_ = state_identifier_in;
  }

  virtual ~StateMachine() {}

  virtual void oneStep() = 0;
  virtual void firstVisit() = 0;
  virtual void lastVisit() = 0;
  virtual bool endOfState() = 0;
  virtual StateIdentifier getNextState() = 0;

  StateIdentifier getStateIdentity() { return state_identity_; }

protected:
  StateIdentifier state_identity_; // Unique integer of this state
  RobotSystem *robot_;             // Pointer to the robot
  double state_machine_time_;
};
