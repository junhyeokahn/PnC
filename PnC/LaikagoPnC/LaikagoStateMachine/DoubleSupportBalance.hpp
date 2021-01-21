#pragma once

#include <Utils/Math/BSplineBasic.h>
#include <PnC/LaikagoPnC/LaikagoStateProvider.hpp>
#include <PnC/StateMachine.hpp>

class LaikagoControlArchitecture;
class LaikagoTaskAndForceContainer;

class DoubleSupportBalance : public StateMachine {
 public:
  DoubleSupportBalance(const StateIdentifier state_identifier_in,
                       LaikagoControlArchitecture* _ctrl_arch,
                       RobotSystem* _robot);
  ~DoubleSupportBalance();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  void initialization(const YAML::Node& node);
  StateIdentifier getNextState();

  void switchStateButtonTrigger() { state_switch_button_trigger_ = true; }

 protected:
  LaikagoStateProvider* sp_;
  LaikagoControlArchitecture* ctrl_arch_;
  LaikagoTaskAndForceContainer* taf_container_;

  double ctrl_start_time_;
  double end_time_;

  bool state_switch_button_trigger_;

  void _taskUpdate();
};
