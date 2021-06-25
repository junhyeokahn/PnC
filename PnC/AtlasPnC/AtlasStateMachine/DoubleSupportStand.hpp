#pragma once

#include <PnC/AtlasPnC/AtlasControlArchitecture.hpp>
#include <PnC/AtlasPnC/AtlasController.hpp>
#include <PnC/AtlasPnC/AtlasStateProvider.hpp>
#include <PnC/StateMachine.hpp>

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
