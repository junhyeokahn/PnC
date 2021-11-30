#pragma once

#include <Eigen/Dense>
#include <zmq.hpp>

#include "messages/draco.pb.h"

class DracoData;

class DracoDataManager {
public:
  static DracoDataManager *GetDracoDataManager();
  virtual ~DracoDataManager();

  void InitializeSockets(const std::string &_addr);

  void Send();

  std::unique_ptr<DracoData> data;

private:
  /* data */
  DracoDataManager();

  std::unique_ptr<zmq::context_t> context_;
  std::unique_ptr<zmq::socket_t> socket_;

  bool b_socket_initialized_;
};

class DracoData {
public:
  DracoData(){};
  ~DracoData(){};

  // should be matching with protobuf msg
  double time = 0.;
  int phase = 0;

  Eigen::VectorXd task_com_pos = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_com_vel = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_com_pos_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_com_vel_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_com_acc_des = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd task_com_pos_local = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_com_vel_local = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_com_pos_des_local = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_com_vel_des_local = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_com_acc_des_local = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd task_cam_vel = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_cam_vel_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_cam_acc_des = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd task_cam_vel_local = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_cam_vel_des_local = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_cam_acc_des_local = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd task_torso_ori_pos_des = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd task_torso_ori_vel_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_torso_ori_acc_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_torso_ori_pos = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd task_torso_ori_vel = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd task_torso_ori_pos_des_local = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd task_torso_ori_vel_des_local = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_torso_ori_acc_des_local = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_torso_ori_pos_local = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd task_torso_ori_vel_local = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd task_rfoot_lin_pos = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_rfoot_lin_vel = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_rfoot_lin_pos_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_rfoot_lin_vel_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_rfoot_lin_acc_des = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd task_rfoot_lin_pos_local = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_rfoot_lin_vel_local = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_rfoot_lin_pos_des_local = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_rfoot_lin_vel_des_local = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_rfoot_lin_acc_des_local = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd task_rfoot_ori_pos_des = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd task_rfoot_ori_vel_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_rfoot_ori_acc_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_rfoot_ori_pos = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd task_rfoot_ori_vel = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd task_rfoot_ori_pos_des_local = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd task_rfoot_ori_vel_des_local = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_rfoot_ori_acc_des_local = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_rfoot_ori_pos_local = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd task_rfoot_ori_vel_local = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd task_lfoot_lin_pos = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_lfoot_lin_vel = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_lfoot_lin_pos_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_lfoot_lin_vel_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_lfoot_lin_acc_des = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd task_lfoot_lin_pos_local = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_lfoot_lin_vel_local = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_lfoot_lin_pos_des_local = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_lfoot_lin_vel_des_local = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_lfoot_lin_acc_des_local = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd task_lfoot_ori_pos_des = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd task_lfoot_ori_vel_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_lfoot_ori_acc_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_lfoot_ori_pos = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd task_lfoot_ori_vel = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd task_lfoot_ori_pos_des_local = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd task_lfoot_ori_vel_des_local = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_lfoot_ori_acc_des_local = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_lfoot_ori_pos_local = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd task_lfoot_ori_vel_local = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd task_upper_body_pos_des = Eigen::VectorXd::Zero(13);
  Eigen::VectorXd task_upper_body_vel_des = Eigen::VectorXd::Zero(13);
  Eigen::VectorXd task_upper_body_acc_des = Eigen::VectorXd::Zero(13);
  Eigen::VectorXd task_upper_body_pos = Eigen::VectorXd::Zero(13);
  Eigen::VectorXd task_upper_body_vel = Eigen::VectorXd::Zero(13);

  // wbc cmd
  Eigen::VectorXd cmd_rfoot_rf = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd cmd_lfoot_rf = Eigen::VectorXd::Zero(6);

  Eigen::VectorXd cmd_joint_positions = Eigen::VectorXd::Zero(27);
  Eigen::VectorXd cmd_joint_velocities = Eigen::VectorXd::Zero(27);
  Eigen::VectorXd cmd_joint_torques = Eigen::VectorXd::Zero(27);
  Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(27);

  // for meshcat visualization
  Eigen::VectorXd joint_positions = Eigen::VectorXd::Zero(27);
  Eigen::VectorXd base_joint_pos = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd base_joint_quat = Eigen::VectorXd::Zero(4);

  // filtered data
  Eigen::VectorXd com_vel_est = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd com_vel_raw = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd imu_ang_vel_est = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd imu_ang_vel_raw = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd cam_est = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd cam_raw = Eigen::VectorXd::Zero(3);

  // icp
  Eigen::VectorXd icp = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd icp_des = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd icp_dot = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd icp_dot_des = Eigen::VectorXd::Zero(2);
};
