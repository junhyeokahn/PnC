#pragma once

#include <Utils/Math/BSplineBasic.h>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/StateMachine.hpp>

class DracoControlArchitecture;
class DracoTaskAndForceContainer;

class ContactTransitionStart : public StateMachine {
 public:
  ContactTransitionStart(const StateIdentifier state_identifier_in,
                         const int _leg_side,
                         DracoControlArchitecture* _ctrl_arch,
                         RobotSystem* _robot);
  ~ContactTransitionStart();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  void initialization(const YAML::Node& node);
  StateIdentifier getNextState();

 protected:
  DracoStateProvider* sp_;
  DracoControlArchitecture* ctrl_arch_;
  DracoTaskAndForceContainer* taf_container_;

  int leg_side_;
  bool final_step_;

  double ctrl_start_time_;
  double end_time_;

  void _taskUpdate();
};
