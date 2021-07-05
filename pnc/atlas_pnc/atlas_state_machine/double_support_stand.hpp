#pragma once

#include <pnc/atlas_pnc/atlas_control_architecture.hpp>
#include <pnc/atlas_pnc/atlas_controller.hpp>
#include <pnc/atlas_pnc/atlas_state_provider.hpp>
#include <pnc/state_machine.hpp>

class DoubleSupportStand : public StateMachine {
public:
  DoubleSupportStand(const StateIdentifier state_identifier_in,
                     AtlasControlArchitecture *_ctrl_arch, RobotSystem *_robot);
  ~DoubleSupportStand();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  StateIdentifier getNextState();

  double end_time;
  double rf_z_max_time;
  double com_height_des;

protected:
  AtlasStateProvider *sp_;
  AtlasControlArchitecture *atlas_ctrl_arch_;

  double ctrl_start_time_;
};
