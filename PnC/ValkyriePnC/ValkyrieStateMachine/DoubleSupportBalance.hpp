#pragma once

#include <Utils/Math/BSplineBasic.h>
#include <PnC/StateMachine.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>

class ValkyrieControlArchitecture;
class ValkyrieTaskAndForceContainer;
class ValkyrieMainController;

class DoubleSupportBalance : public StateMachine {
 public:
  DoubleSupportBalance(const StateIdentifier state_identifier_in,
                       ValkyrieControlArchitecture* _ctrl_arch,
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
  ValkyrieStateProvider* sp_;
  ValkyrieControlArchitecture* val_ctrl_arch_;
  ValkyrieTaskAndForceContainer* taf_container_;

  double ctrl_start_time_;
  double end_time_;

  bool state_switch_button_trigger_;

  void _taskUpdate();
};
