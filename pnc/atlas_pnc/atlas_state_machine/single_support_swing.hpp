#pragma once

#include <pnc/atlas_pnc/atlas_control_architecture.hpp>
#include <pnc/atlas_pnc/atlas_controller.hpp>
#include <pnc/atlas_pnc/atlas_state_provider.hpp>
#include <pnc/state_machine.hpp>

class SingleSupportSwing : public StateMachine {
public:
  SingleSupportSwing(const StateIdentifier state_identifier_in,
                     AtlasControlArchitecture *_ctrl_arch, int _leg_side,
                     RobotSystem *_robot);
  ~SingleSupportSwing();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  StateIdentifier getNextState();

protected:
  AtlasStateProvider *sp_;
  AtlasControlArchitecture *atlas_ctrl_arch_;

  double ctrl_start_time_;
  int leg_side_;

  double end_time_;
};
