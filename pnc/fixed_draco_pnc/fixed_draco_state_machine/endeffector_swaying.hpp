#pragma once

#include "pnc/state_machine.hpp"
#include "pnc/draco_pnc/draco_control_architecture.hpp"
#include "pnc/draco_pnc/draco_controller.hpp"
#include "pnc/draco_pnc/draco_state_provider.hpp"

class EndEffectorSwaying : public StateMachine {
public:
  EndEffectorSwaying(const StateIdentifier state_identifier_in,
                     DracoControlArchitecture *_ctrl_arch, int _endeffector_side,
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
  DracoStateProvider *sp_;
  FixedDracoControlArchitecture *ctrl_arch_;

  double ctrl_start_time_;
  int leg_side_;

};
