#pragma once

#include "pnc/draco_pnc/draco_control_architecture.hpp"
#include "pnc/draco_pnc/draco_state_provider.hpp"
#include "pnc/state_machine.hpp"

class HandReaching : public StateMachine {
public:
  HandReaching(const StateIdentifier _state_identifier_in,
               DracoControlArchitecture *_ctrl_arch, RobotSystem *_robot);
  ~HandReaching() = default;

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  StateIdentifier getNextState();

  void setDuration(double _duration) { duration_ = _duration; }
  void setRelTargetPos(const Eigen::Vector3d &pos) { rel_target_pos_ = pos; }
  void setRelTargetOri(const Eigen::Quaterniond &ori) { rel_target_ori_ = ori; }

  bool b_trigger_return_;

protected:
  DracoStateProvider *sp_;
  DracoControlArchitecture *ctrl_arch_;

  double duration_;
  Eigen::Vector3d rel_target_pos_;
  Eigen::Quaterniond rel_target_ori_;
};
