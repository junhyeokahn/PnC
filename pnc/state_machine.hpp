#pragma once

#include "pnc/robot_system/robot_system.hpp"
#include "pnc/whole_body_controllers/contact.hpp"
#include "pnc/whole_body_controllers/task.hpp"
#include "utils/util.hpp"

typedef int StateIdentifier;

/// class StateMachine
class StateMachine {
public:
  /// \{ \name Constructor and Destructor
  StateMachine(const StateIdentifier state_identifier_in, RobotSystem *_robot) {
    robot_ = _robot;
    state_machine_time_ = 0.;
    state_identity_ = state_identifier_in;
  }

  virtual ~StateMachine() {}
  /// \}

  /// Compute commands in this state.
  virtual void oneStep() = 0;

  /// Process in the first visit in this state. This should be called at the
  /// beginning of this state.
  virtual void firstVisit() = 0;

  /// Process in the last visit in this state. This should be called at the last
  /// of this state.
  virtual void lastVisit() = 0;

  /// Return whether this state ends or not.
  virtual bool endOfState() = 0;

  /// Return StateIdentifier of the next state.
  virtual StateIdentifier getNextState() = 0;

  /// Return StateIdentifier of this state.
  StateIdentifier getStateIdentity() { return state_identity_; }

protected:
  StateIdentifier state_identity_;
  RobotSystem *robot_;
  double state_machine_time_;
};
