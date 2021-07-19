#pragma once

#include <pnc/state_machine.hpp>
#include <pnc/draco_pnc/draco_control_architecture.hpp>
#include <pnc/draco_pnc/draco_controller.hpp>
#include <pnc/draco_pnc/draco_state_provider.hpp>

class ContactTransitionEnd : public StateMachine {
public:
  ContactTransitionEnd(const StateIdentifier state_identifier_in,
                       DracoControlArchitecture *_ctrl_arch, int _leg_side,
                       RobotSystem *_robot);
  ~ContactTransitionEnd();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  StateIdentifier getNextState();

protected:
  DracoStateProvider *sp_;
  DracoControlArchitecture *ctrl_arch_;

  double ctrl_start_time_;
  int leg_side_;

  double end_time_;
};
