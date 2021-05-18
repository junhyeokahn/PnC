#pragma once

#include <Utils/Math/BSplineBasic.h>
#include <PnC/A1PnC/A1StateProvider.hpp>
#include <PnC/StateMachine.hpp>

class A1ControlArchitecture;
class A1TaskAndForceContainer;

class QuadSupportBalance : public StateMachine {
 public:
  QuadSupportBalance(const StateIdentifier state_identifier_in,
                       A1ControlArchitecture* _ctrl_arch,
                       RobotSystem* _robot);
  ~QuadSupportBalance();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  void initialization(const YAML::Node& node);
  StateIdentifier getNextState();
  void updateTestCounter(int _counter) {}

  void switchStateButtonTrigger() { state_switch_button_trigger_ = true; }

 protected:
  A1StateProvider* sp_;
  A1ControlArchitecture* ctrl_arch_;
  A1TaskAndForceContainer* taf_container_;

  double ctrl_start_time_;
  double end_time_;

  bool state_switch_button_trigger_;

  void _taskUpdate();
};
