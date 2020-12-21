#pragma once

#include <Utils/Math/BSplineBasic.h>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/StateMachine.hpp>

class DracoControlArchitecture;
class DracoTaskAndForceContainer;

class DoubleSupportStand : public StateMachine {
 public:
  DoubleSupportStand(const StateIdentifier state_identifier_in,
                     DracoControlArchitecture* _ctrl_arch, RobotSystem* _robot);
  ~DoubleSupportStand();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  void initialization(const YAML::Node& node);
  double progression_variable() { return state_machine_time_ / smoothing_dur_; }
  StateIdentifier getNextState();

 protected:
  DracoStateProvider* sp_;
  DracoControlArchitecture* ctrl_arch_;
  DracoTaskAndForceContainer* taf_container_;

  double ctrl_start_time_;
  double smoothing_dur_;
  double end_time_;
  double time_to_max_normal_force_;
  double target_height_;

  int contact_type_;

  void _taskUpdate();
};
