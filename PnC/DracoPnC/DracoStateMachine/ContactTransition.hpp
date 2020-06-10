#pragma once

#include <Utils/Math/BSplineBasic.h>
#include <PnC/StateMachine.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>

class DracoControlArchitecture;
class DracoTaskAndForceContainer;

class ContactTransition : public StateMachine {
 public:
  ContactTransition(const StateIdentifier state_identifier_in,
                    const int _leg_side, DracoControlArchitecture* _ctrl_arch,
                    RobotSystem* _robot);
  ~ContactTransition();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  void initialization(const YAML::Node& node);
  StateIdentifier getNextState();

 protected:
  ValkyrieStateProvider* sp_;
  DracoControlArchitecture* ctrl_arch_;
  DracoTaskAndForceContainer* taf_container_;

  int leg_side_;
  bool final_step_;

  double ctrl_start_time_;
  double end_time_;

  void _taskUpdate();
};
