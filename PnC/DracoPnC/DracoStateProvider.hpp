#pragma once

#include <Utils/Utilities.hpp>
#include <Configuration.h>

class RobotSystem;

class DracoStateProvider{
public:
  static DracoStateProvider* getStateProvider(RobotSystem* _robot);
  ~DracoStateProvider(){}

  std::string stance_foot;
  double curr_time;

  Eigen::VectorXd q;
  Eigen::VectorXd qdot;
  Eigen::VectorXd rotor_inertia;

  Eigen::Vector3d global_pos_local;
  Eigen::Vector2d des_location;
  Eigen::Vector2d est_mocap_body_vel;
  Eigen::VectorXd des_jpos_prev;

  int b_rfoot_contact;
  int b_lfoot_contact;

  Eigen::VectorXd qddot_cmd;
  Eigen::VectorXd reaction_forces;

  Eigen::Vector3d rfoot_pos;
  Eigen::Vector3d lfoot_pos;
  Eigen::Vector3d rfoot_vel;
  Eigen::Vector3d lfoot_vel;

  Eigen::VectorXd led_kin_data;

  Eigen::Vector3d com_pos;
  Eigen::Vector3d com_vel;
  Eigen::Vector3d est_com_vel;

  int num_step_copy;
  int phase_copy;

  void saveCurrentData();
private:
  DracoStateProvider(RobotSystem* _robot);
  RobotSystem* robot_;
};
