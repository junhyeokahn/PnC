#pragma once

#include <PnC/AtlasPnC/AtlasControlArchitecture.hpp>
#include <PnC/AtlasPnC/AtlasController.hpp>
#include <PnC/AtlasPnC/AtlasStateProvider.hpp>
#include <PnC/StateMachine.hpp>

class ContactTransitionStart : public StateMachine {
public:
  ContactTransitionStart(const StateIdentifier state_identifier_in,
                         AtlasControlArchitecture *_ctrl_arch, int _leg_side,
                         RobotSystem *_robot);
  ~ContactTransitionStart();

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
