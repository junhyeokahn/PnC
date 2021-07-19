#pragma once

#include "pnc/draco_pnc/draco_control_architecture.hpp"
#include "pnc/draco_pnc/draco_controller.hpp"
#include "pnc/draco_pnc/draco_state_provider.hpp"
#include "pnc/state_machine.hpp"

class DoubleSupportInterpolation : public StateMachine {
public:
  DoubleSupportInterpolation(const StateIdentifier state_identifier_in,
                             DracoControlArchitecture *_ctrl_arch,
                             RobotSystem *_robot);
  ~DoubleSupportInterpolation();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  StateIdentifier getNextState();

  Eigen::Vector3d local_offset;
  double end_time;

protected:
  DracoStateProvider *sp_;
  DracoControlArchitecture *ctrl_arch_;

  double ctrl_start_time_;
};
