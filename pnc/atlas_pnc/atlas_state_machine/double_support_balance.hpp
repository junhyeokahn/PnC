#pragma once

#include <pnc/atlas_pnc/atlas_control_architecture.hpp>
#include <pnc/atlas_pnc/atlas_controller.hpp>
#include <pnc/atlas_pnc/atlas_state_provider.hpp>
#include <pnc/state_machine.hpp>

class DoubleSupportBalance : public StateMachine {
public:
  DoubleSupportBalance(const StateIdentifier state_identifier_in,
                       AtlasControlArchitecture *_ctrl_arch,
                       RobotSystem *_robot);
  ~DoubleSupportBalance();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  StateIdentifier getNextState();

  bool b_state_switch_trigger;

protected:
  AtlasStateProvider *sp_;
  AtlasControlArchitecture *atlas_ctrl_arch_;

  double ctrl_start_time_;
};
