#pragma once

#include <Utils/Math/BSplineBasic.h>
#include <PnC/A1PnC/A1StateProvider.hpp>
#include <PnC/StateMachine.hpp>

class A1ControlArchitecture;
class A1TaskAndForceContainer;
class A1MainController;

class ContactTransitionEnd : public StateMachine {
 public:
  ContactTransitionEnd(const StateIdentifier state_identifier_in,
                       const int _leg_side,
                       A1ControlArchitecture* _ctrl_arch,
                       RobotSystem* _robot);
  ~ContactTransitionEnd();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  void initialization(const YAML::Node& node);
  StateIdentifier getNextState();
  void updateTestCounter(int _counter) {}

 protected:
  A1StateProvider* sp_;
  A1ControlArchitecture* ctrl_arch_;
  A1TaskAndForceContainer* taf_container_;

  int leg_side_;
  bool final_step_;

  double ctrl_start_time_;
  double end_time_;
  double ramp_time_;

  void _taskUpdate();
};
