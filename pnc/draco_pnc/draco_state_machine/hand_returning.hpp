#pragma once

#include "pnc/draco_pnc/draco_control_architecture.hpp"
#include "pnc/draco_pnc/draco_state_provider.hpp"
#include "pnc/state_machine.hpp"

class HandReturning : public StateMachine {
public:
  HandReturning(const StateIdentifier _state_identifier_in,
                DracoControlArchitecture *_ctrl_arch, RobotSystem *_robot);
  ~HandReturning() = default;

  void firstVisit();
  void oneStep();
  bool endOfState();
  void lastVisit();
  StateIdentifier getNextState();

  void setDuration(double _duration) { duration_ = _duration; }

protected:
  DracoControlArchitecture *ctrl_arch_;
  DracoStateProvider *sp_;

  double duration_;
  double ctrl_start_time_;
};
