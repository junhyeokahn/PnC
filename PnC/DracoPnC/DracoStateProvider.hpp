#pragma once

#include <Configuration.h>
#include <PnC/Planner/towr_plus/include/towr_plus/locomotion_solution.h>
#include <Utils/IO/IOUtilities.hpp>

class RobotSystem;

class DracoStateProvider {
 public:
  static DracoStateProvider* getStateProvider(RobotSystem* _robot);
  ~DracoStateProvider() { delete one_step_sol; }

  LocomotionSolution* one_step_sol;
  double planning_time;

  // ---------------------------------------------------------------------------
  // Variables set outside
  // ---------------------------------------------------------------------------
  double curr_time;

  int stance_foot;
  int prev_stance_foot;

  Eigen::VectorXd rotor_inertia;
  Eigen::VectorXd q;
  Eigen::VectorXd qdot;
  Eigen::VectorXd q_task_des;
  Eigen::VectorXd qdot_task_des;
  Eigen::VectorXd q_task;
  Eigen::VectorXd qdot_task;
  Eigen::VectorXd prev_trq_cmd;

  Eigen::Vector3d com_pos_des;
  Eigen::Vector3d com_vel_des;
  Eigen::Vector3d com_pos;
  Eigen::Vector3d com_vel;
  Eigen::Vector3d est_com_vel;
  Eigen::Quaternion<double> base_quat_des;
  Eigen::Vector3d base_ang_vel_des;
  Eigen::Quaternion<double> base_quat;
  Eigen::Vector3d base_ang_vel;

  int b_rfoot_contact;
  int b_lfoot_contact;

  Eigen::VectorXd qddot_cmd;

  Eigen::Vector3d dcm;
  Eigen::Vector3d dcm_des;
  Eigen::Vector3d prev_dcm;
  Eigen::Vector3d dcm_vel;
  Eigen::Vector3d dcm_vel_des;
  Eigen::Vector3d r_vrp;
  Eigen::Vector3d r_vrp_des;

  /*  Eigen::Vector3d r_rf_front_des;*/
  // Eigen::Vector3d r_rf_back_des;
  // Eigen::Vector3d l_rf_front_des;
  /*Eigen::Vector3d l_rf_back_des;*/
  Eigen::VectorXd r_rf_des;
  Eigen::VectorXd l_rf_des;
  Eigen::VectorXd r_rf;
  Eigen::VectorXd l_rf;

  int phase_copy;

  int planning_id;

  Eigen::Vector3d rfoot_center_pos_des;
  Eigen::Vector3d lfoot_center_pos_des;
  Eigen::Vector3d rfoot_center_vel_des;
  Eigen::Vector3d lfoot_center_vel_des;
  Eigen::Vector3d rfoot_center_so3_des;
  Eigen::Vector3d lfoot_center_so3_des;
  Eigen::Quaternion<double> rfoot_center_quat_des;
  Eigen::Quaternion<double> lfoot_center_quat_des;

  double w_rfoot_pos;
  double w_rfoot_ori;
  double w_lfoot_pos;
  double w_lfoot_ori;
  double w_com;
  double w_base_ori;
  /*  double w_rf_rffront;*/
  // double w_rf_rfback;
  // double w_rf_lffront;
  /*double w_rf_lfback;*/
  double w_rfoot_fr;
  double w_lfoot_fr;

  // ---------------------------------------------------------------------------
  // Variables set here
  // ---------------------------------------------------------------------------
  Eigen::Vector3d rfoot_center_pos;
  Eigen::Vector3d lfoot_center_pos;
  Eigen::Vector3d rfoot_center_vel;
  Eigen::Vector3d lfoot_center_vel;
  Eigen::Vector3d rfoot_center_so3;
  Eigen::Vector3d lfoot_center_so3;
  Eigen::Quaternion<double> rfoot_center_quat;
  Eigen::Quaternion<double> lfoot_center_quat;

  double dcm_omega;

  void saveCurrentData();

 private:
  DracoStateProvider(RobotSystem* _robot);
  RobotSystem* robot_;
};
