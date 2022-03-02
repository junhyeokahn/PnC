#pragma once

#include "pnc/draco_pnc/draco_control_architecture.hpp"
#include "pnc/draco_pnc/draco_controller.hpp"
#include "pnc/draco_pnc/draco_state_provider.hpp"
#include "pnc/state_machine.hpp"

class SingleSupportLifting : public StateMachine {
public:
  SingleSupportLifting(const StateIdentifier state_identifier,
                       DracoControlArchitecture *_ctrl_arch, int _leg_side,
                       RobotSystem *_robot);
  ~SingleSupportLifting();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  StateIdentifier getNextState();

  double moving_duration_;
  double des_foot_z_pos_; // Assume flat terrain; global increment

  bool b_static_balancing_trigger;

protected:
  DracoStateProvider *sp_;
  DracoControlArchitecture *ctrl_arch_;

  double ctrl_start_time_;
  int leg_side_;
};
