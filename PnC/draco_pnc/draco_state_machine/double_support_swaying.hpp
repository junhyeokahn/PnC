#pragma once

#include <PnC/StateMachine.hpp>
#include <PnC/draco_pnc/draco_control_architecture.hpp>
#include <PnC/draco_pnc/draco_controller.hpp>
#include <PnC/draco_pnc/draco_state_provider.hpp>

class DoubleSupportSwaying : public StateMachine {
public:
  DoubleSupportSwaying(const StateIdentifier state_identifier_in,
                       DracoControlArchitecture *_ctrl_arch,
                       RobotSystem *_robot);
  ~DoubleSupportSwaying();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  StateIdentifier getNextState();

  double end_time;
  double rf_z_max_time;
  double com_height_des;

  Eigen::Vector3d amp;
  Eigen::Vector3d freq;

protected:
  DracoStateProvider *sp_;
  DracoControlArchitecture *ctrl_arch_;

  double ctrl_start_time_;
};
