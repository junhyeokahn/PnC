#pragma once

#include <Utils/Math/BSplineBasic.h>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/StateMachine.hpp>

class A1ControlArchitecture;
class A1TaskAndForceContainer;

class Initialize : public StateMachine {
 public:
  Initialize(const StateIdentifier state_identifier_in,
             A1ControlArchitecture* _ctrl_arch,
             RobotSystem* _robot);
  ~Initialize();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  void initialization(const YAML::Node& node);
  StateIdentifier getNextState();

  void switchStateButtonTrigger() { state_switch_button_trigger_ = true; }

 protected:
  A1StateProvider* sp_;
  A1ControlArchitecture* ctrl_arch_;
  A1TaskAndForceContainer* taf_container_;

  double ctrl_start_time_;
  double end_time_;
  Eigen::VectorXd target_pos_;

  void _taskUpdate();
};
