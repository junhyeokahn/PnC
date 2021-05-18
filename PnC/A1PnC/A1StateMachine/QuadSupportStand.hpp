#pragma once

#include <Utils/Math/BSplineBasic.h>
#include <PnC/A1PnC/A1StateProvider.hpp>
#include <PnC/StateMachine.hpp>

class A1ControlArchitecture;
class A1TaskAndForceContainer;

class QuadSupportStand : public StateMachine {
 public:
  QuadSupportStand(const StateIdentifier state_identifier_in,
                     A1ControlArchitecture* _ctrl_arch, RobotSystem* _robot);
  ~QuadSupportStand();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  void initialization(const YAML::Node& node);
  double progression_variable() { return state_machine_time_ / smoothing_dur_; }
  StateIdentifier getNextState();

  void updateTestCounter(int _counter) {}

 protected:
  A1StateProvider* sp_;
  A1ControlArchitecture* ctrl_arch_;
  A1TaskAndForceContainer* taf_container_;

  double ctrl_start_time_;
  double smoothing_dur_;
  double end_time_;
  double time_to_max_normal_force_;
  double target_height_;

  void _taskUpdate();
};
