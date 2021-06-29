#pragma once

#include "PnC/StateMachine.hpp"
#include "PnC/draco_pnc/draco_control_architecture.hpp"
#include "PnC/draco_pnc/draco_controller.hpp"
#include "PnC/draco_pnc/draco_state_provider.hpp"

class DoubleSupportBalance : public StateMachine {
public:
  DoubleSupportBalance(const StateIdentifier state_identifier_in,
                       DracoControlArchitecture *_ctrl_arch,
                       RobotSystem *_robot);
  ~DoubleSupportBalance();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  StateIdentifier getNextState();

  bool b_state_switch_trigger;

protected:
  DracoStateProvider *sp_;
  DracoControlArchitecture *ctrl_arch_;

  double ctrl_start_time_;
};
