#pragma once

#include "pnc/fixed_draco_pnc/fixed_draco_control_architecture.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_controller.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_provider.hpp"
#include "pnc/state_machine.hpp"

class EndEffectorHold : public StateMachine {
public:
  EndEffectorHold(const StateIdentifier state_identifier_in,
                  FixedDracoControlArchitecture *_ctrl_arch,
                  RobotSystem *_robot);
  ~EndEffectorHold();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  StateIdentifier getNextState();

  bool b_rf_swaying_trigger;
  bool b_lf_swaying_trigger;
  bool b_rh_swaying_trigger;
  bool b_lh_swaying_trigger;

protected:
  FixedDracoStateProvider *sp_;
  FixedDracoControlArchitecture *ctrl_arch_;

  Eigen::Isometry3d target_rf_iso_;
  Eigen::Isometry3d target_lf_iso_;
  Eigen::Isometry3d target_rh_iso_;
  Eigen::Isometry3d target_lh_iso_;

  double ctrl_start_time_;
};
