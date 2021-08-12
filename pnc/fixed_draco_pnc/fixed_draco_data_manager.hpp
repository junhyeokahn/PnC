#pragma once

#include <Eigen/Dense>
#include <zmq.hpp>

#include "messages/fixed_draco.pb.h"

class FixedDracoData;

class FixedDracoDataManager {
public:
  static FixedDracoDataManager *GetFixedDracoDataManager();
  virtual ~FixedDracoDataManager();

  void InitializeSockets(const std::string &_addr);

  void Send();

  std::unique_ptr<FixedDracoData> data;

private:
  /* data */
  FixedDracoDataManager();

  std::unique_ptr<zmq::context_t> context_;
  std::unique_ptr<zmq::socket_t> socket_;

  bool b_socket_initialized_;
};

class FixedDracoData {
public:
  FixedDracoData(){};
  ~FixedDracoData(){};

  // should be matching with protobuf msg
  double time = 0.;
  int phase = 0;

  Eigen::VectorXd task_rfoot_pos = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_rfoot_vel = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_rfoot_pos_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_rfoot_vel_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_rfoot_acc_des = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd task_rfoot_ori_des = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd task_rfoot_ang_vel_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_rfoot_ang_acc_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_rfoot_ori = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd task_rfoot_ang_vel = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd task_lfoot_pos = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_lfoot_vel = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_lfoot_pos_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_lfoot_vel_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_lfoot_acc_des = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd task_lfoot_ori_des = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd task_lfoot_ang_vel_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_lfoot_ang_acc_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_lfoot_ori = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd task_lfoot_ang_vel = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd task_rhand_pos = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_rhand_vel = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_rhand_pos_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_rhand_vel_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_rhand_acc_des = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd task_rhand_ori_des = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd task_rhand_ang_vel_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_rhand_ang_acc_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_rhand_ori = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd task_rhand_ang_vel = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd task_lhand_pos = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_lhand_vel = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_lhand_pos_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_lhand_vel_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_lhand_acc_des = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd task_lhand_ori_des = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd task_lhand_ang_vel_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_lhand_ang_acc_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd task_lhand_ori = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd task_lhand_ang_vel = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd task_neck_pos_des = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd task_neck_vel_des = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd task_neck_acc_des = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd task_neck_pos = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd task_neck_vel = Eigen::VectorXd::Zero(1);

  // Eigen::VectorXd task_upper_body_pos_des = Eigen::VectorXd::Zero(13);
  // Eigen::VectorXd task_upper_body_vel_des = Eigen::VectorXd::Zero(13);
  // Eigen::VectorXd task_upper_body_acc_des = Eigen::VectorXd::Zero(13);
  // Eigen::VectorXd task_upper_body_pos = Eigen::VectorXd::Zero(13);
  // Eigen::VectorXd task_upper_body_vel = Eigen::VectorXd::Zero(13);

  // wbc cmd
  Eigen::VectorXd cmd_joint_positions = Eigen::VectorXd::Zero(27);
  Eigen::VectorXd cmd_joint_velocities = Eigen::VectorXd::Zero(27);
  Eigen::VectorXd cmd_joint_torques = Eigen::VectorXd::Zero(27);
  Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(27);

  // for meshcat visualization
  Eigen::VectorXd joint_positions = Eigen::VectorXd::Zero(27);
  Eigen::VectorXd base_joint_pos = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd base_joint_quat = Eigen::VectorXd::Zero(4);
};
