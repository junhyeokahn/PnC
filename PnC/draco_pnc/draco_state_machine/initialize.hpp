#pragma once

#include "PnC/StateMachine.hpp"
#include "PnC/draco_pnc/draco_control_architecture.hpp"
#include "PnC/draco_pnc/draco_controller.hpp"
#include "PnC/draco_pnc/draco_state_provider.hpp"

class Initialize : public StateMachine {
public:
  Initialize(const StateIdentifier state_identifier_in,
             DracoControlArchitecture *_ctrl_arch, RobotSystem *_robot);
  ~Initialize();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  StateIdentifier getNextState();

  double end_time;

  Eigen::VectorXd target_jpos;

protected:
  DracoStateProvider *sp_;
  DracoControlArchitecture *ctrl_arch_;

  double ctrl_start_time_;

  Eigen::VectorXd initial_jpos_;
};
