#pragma once

#include "pnc/fixed_draco_pnc/fixed_draco_control_architecture.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_controller.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_provider.hpp"
#include "pnc/state_machine.hpp"

class EndEffectorSwaying : public StateMachine {
public:
  EndEffectorSwaying(const StateIdentifier state_identifier_in,
                     FixedDracoControlArchitecture *_ctrl_arch, int _leg_side,
                     RobotSystem *_robot);
  ~EndEffectorSwaying();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  StateIdentifier getNextState();

  Eigen::Vector3d amp;
  Eigen::Vector3d freq;

protected:
  FixedDracoStateProvider *sp_;
  FixedDracoControlArchitecture *ctrl_arch_;

  Eigen::Isometry3d target_rf_iso_;
  Eigen::Isometry3d target_lf_iso_;

  double ctrl_start_time_;
  int leg_side_;
};
