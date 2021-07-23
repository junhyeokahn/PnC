#pragma once

#include "pnc/state_machine.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_control_architecture.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_controller.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_provider.hpp"

class Initialize : public StateMachine {
public:
  Initialize(const StateIdentifier state_identifier_in,
             FixedDracoControlArchitecture *_ctrl_arch, RobotSystem *_robot);
  ~Initialize();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  StateIdentifier getNextState();

  double end_time;

  Eigen::VectorXd target_jpos;

protected:
  FixedDracoStateProvider *sp_;
  FixedDracoControlArchitecture *ctrl_arch_;

  double ctrl_start_time_;

  Eigen::VectorXd initial_jpos_;
};
