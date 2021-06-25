#pragma once

#include <PnC/AtlasPnC/AtlasControlArchitecture.hpp>
#include <PnC/AtlasPnC/AtlasController.hpp>
#include <PnC/AtlasPnC/AtlasStateProvider.hpp>
#include <PnC/StateMachine.hpp>

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
