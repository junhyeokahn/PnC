#pragma once

#include <Utils/Math/BSplineBasic.h>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/StateMachine.hpp>

class DracoControlArchitecture;
class DracoTaskAndForceContainer;

class SwingControl : public StateMachine {
 public:
  SwingControl(const StateIdentifier state_identifier_in, const int _leg_side,
               DracoControlArchitecture* _ctrl_arch, RobotSystem* _robot);
  ~SwingControl();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  void initialization(const YAML::Node& node);
  StateIdentifier getNextState();

 protected:
  DracoStateProvider* sp_;
  DracoControlArchitecture* val_ctrl_arch_;
  DracoTaskAndForceContainer* taf_container_;

  int leg_side_;

  double ctrl_start_time_;
  double end_time_;

  double swing_time_percent_early_contact_check_;
  double early_contact_force_threshold_;

  void _taskUpdate();
};
