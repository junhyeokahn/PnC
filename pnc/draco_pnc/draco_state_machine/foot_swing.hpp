#pragma once

#include "pnc/draco_pnc/draco_control_architecture.hpp"
#include "pnc/draco_pnc/draco_controller.hpp"
#include "pnc/draco_pnc/draco_state_provider.hpp"
#include "pnc/state_machine.hpp"

class FootSwing : public StateMachine {
public:
  FootSwing(const StateIdentifier state_identifier_in,
            DracoControlArchitecture *_ctrl_arch, int _leg_side,
            RobotSystem *_robot);
  ~FootSwing();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  StateIdentifier getNextState();

  double swing_duration_;
  double des_foot_x_increment_;   // local value
  double des_foot_y_increment_;   // local value
  double des_foot_ori_increment_; // local yaw value (in degree)

  bool b_static_walking_trigger;

protected:
  DracoStateProvider *sp_;
  DracoControlArchitecture *ctrl_arch_;

  double ctrl_start_time_;
  int leg_side_;

  Eigen::Isometry3d des_foot_iso_;
};
