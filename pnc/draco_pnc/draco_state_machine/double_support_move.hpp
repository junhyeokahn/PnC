#pragma once

#include "pnc/draco_pnc/draco_control_architecture.hpp"
#include "pnc/draco_pnc/draco_controller.hpp"
#include "pnc/draco_pnc/draco_state_provider.hpp"
#include "pnc/state_machine.hpp"

namespace com_move_states {
constexpr int Left = 0;
constexpr int Right = 1;
constexpr int Center = 2;
} // namespace com_states

class DoubleSupportMove : public StateMachine {
public:
  DoubleSupportMove(const StateIdentifier state_identifier_in,
                       DracoControlArchitecture *_ctrl_arch,
                       int _com_states,
                       RobotSystem *_robot);
  ~DoubleSupportMove();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  StateIdentifier getNextState();

  double duration_;

protected:
  DracoStateProvider *sp_;
  DracoControlArchitecture *ctrl_arch_;

  int com_move_states_;

  double ctrl_start_time_;
};
