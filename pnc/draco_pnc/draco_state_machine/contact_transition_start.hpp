#pragma once

#include "pnc/draco_pnc/draco_control_architecture.hpp"
#include "pnc/draco_pnc/draco_controller.hpp"
#include "pnc/draco_pnc/draco_state_provider.hpp"
#include "pnc/state_machine.hpp"

class ContactTransitionStart : public StateMachine {
public:
  ContactTransitionStart(const StateIdentifier state_identifier_in,
                         DracoControlArchitecture *_ctrl_arch, int _leg_side,
                         RobotSystem *_robot);
  ~ContactTransitionStart();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  StateIdentifier getNextState();

  bool b_use_base_height;

protected:
  DracoStateProvider *sp_;
  DracoControlArchitecture *ctrl_arch_;

  double ctrl_start_time_;
  int leg_side_;

  double end_time_;
};
