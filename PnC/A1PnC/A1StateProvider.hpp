#pragma once

#include <Configuration.h>
#include <Utils/IO/IOUtilities.hpp>

class RobotSystem;

class A1StateProvider {
 public:
  static A1StateProvider* getStateProvider(RobotSystem* _robot);
  ~A1StateProvider() {}

  // ---------------------------------------------------------------------------
  // Variables set outside
  // ---------------------------------------------------------------------------
  double curr_time;

  int front_stance_foot;
  int rear_stance_foot;

  // MPC related
  Eigen::VectorXd mpc_rxn_forces;
  Eigen::VectorXd x_y_yaw_vel_des;
  Eigen::VectorXd interpolated_mpc_forces;
  Eigen::VectorXd final_reaction_forces;

  // Variables for Mingyo Footstep Planner
  Eigen::VectorXd next_front_foot_location;
  Eigen::VectorXd next_rear_foot_location;

  Eigen::VectorXd rotor_inertia;
  Eigen::VectorXd q;
  Eigen::VectorXd qdot;
  Eigen::VectorXd q_task_des;
  Eigen::VectorXd qdot_task_des;
  Eigen::VectorXd q_task;
  Eigen::VectorXd qdot_task;
  Eigen::VectorXd prev_trq_cmd;

  Eigen::VectorXd jpos_ini;

  Eigen::Vector3d com_pos_des;
  Eigen::Vector3d com_vel_des;
  Eigen::Vector3d com_pos;
  Eigen::Vector3d com_vel;
  // Eigen::Vector3d est_com_vel;
  Eigen::Quaternion<double> base_quat_des;
  Eigen::Vector3d base_ang_vel_des;
  Eigen::Quaternion<double> base_quat;
  Eigen::Vector3d base_ang_vel;

  int b_frfoot_contact;
  int b_flfoot_contact;
  int b_rrfoot_contact;
  int b_rlfoot_contact;

  Eigen::VectorXd qddot_cmd;

  int phase_copy;

  int planning_id;

  Eigen::Vector3d frfoot_pos_des;
  Eigen::Vector3d flfoot_pos_des;
  Eigen::Vector3d rrfoot_pos_des;
  Eigen::Vector3d rlfoot_pos_des; 
  Eigen::Vector3d frfoot_vel_des;
  Eigen::Vector3d flfoot_vel_des;
  Eigen::Vector3d rrfoot_vel_des;
  Eigen::Vector3d rlfoot_vel_des;

  double w_frfoot_pos;
  double w_flfoot_pos;
  double w_rrfoot_pos;
  double w_rlfoot_pos;
  double w_com;
  double w_base_ori;
  double w_frfoot_fr;
  double w_flfoot_fr;
  double w_rrfoot_fr;
  double w_rlfoot_fr;

  Eigen::Vector3d flfoot_landing;
  Eigen::Vector3d frfoot_landing;
  Eigen::Vector3d rlfoot_landing;
  Eigen::Vector3d rrfoot_landing;

  // ---------------------------------------------------------------------------
  // Variables set here
  // ---------------------------------------------------------------------------
  Eigen::Vector3d frfoot_pos;
  Eigen::Vector3d flfoot_pos;
  Eigen::Vector3d rrfoot_pos;
  Eigen::Vector3d rlfoot_pos;
  Eigen::Vector3d frfoot_vel;
  Eigen::Vector3d flfoot_vel;
  Eigen::Vector3d rrfoot_vel;
  Eigen::Vector3d rlfoot_vel;

  void saveCurrentData();

 private:
  A1StateProvider(RobotSystem* _robot);
  RobotSystem* robot_;
};
