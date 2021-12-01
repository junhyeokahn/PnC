#include "pnc/draco_pnc/draco_data_manager.hpp"
#include <iostream>

DracoDataManager::DracoDataManager() {
  b_socket_initialized_ = false;
  data = std::make_unique<DracoData>();
}

DracoDataManager::~DracoDataManager() {}

DracoDataManager *DracoDataManager::GetDracoDataManager() {
  static DracoDataManager instance;
  return &instance;
}

void DracoDataManager::InitializeSockets(const std::string &_addr) {
  if (b_socket_initialized_) {
    // skip this
  } else {
    context_ = std::make_unique<zmq::context_t>(1);
    socket_ = std::make_unique<zmq::socket_t>(*context_, ZMQ_PUB);
    socket_->bind(_addr);
    b_socket_initialized_ = true;
  }
}

void DracoDataManager::Send() {
  assert(b_socket_initialized_);

  // copy data to protobuf msg
  draco::pnc_msg pb_msg;

  pb_msg.set_time(data->time);
  pb_msg.set_phase(data->phase);

  pb_msg.set_l_knee_int_frc_cmd(data->l_knee_int_frc_cmd);
  pb_msg.set_r_knee_int_frc_cmd(data->r_knee_int_frc_cmd);

  for (int i = 0; i < 3; ++i) {
    pb_msg.add_task_com_pos(data->task_com_pos[i]);
    pb_msg.add_task_com_vel(data->task_com_vel[i]);
    pb_msg.add_task_com_pos_des(data->task_com_pos_des[i]);
    pb_msg.add_task_com_vel_des(data->task_com_vel_des[i]);
    pb_msg.add_task_com_acc_des(data->task_com_acc_des[i]);

    pb_msg.add_task_com_pos_local(data->task_com_pos_local[i]);
    pb_msg.add_task_com_vel_local(data->task_com_vel_local[i]);
    pb_msg.add_task_com_pos_des_local(data->task_com_pos_des_local[i]);
    pb_msg.add_task_com_vel_des_local(data->task_com_vel_des_local[i]);
    pb_msg.add_task_com_acc_des_local(data->task_com_acc_des_local[i]);

    pb_msg.add_task_cam_vel(data->task_cam_vel[i]);
    pb_msg.add_task_cam_vel_des(data->task_cam_vel_des[i]);
    pb_msg.add_task_cam_acc_des(data->task_cam_acc_des[i]);

    pb_msg.add_task_torso_ori_pos(data->task_torso_ori_pos[i]);
    pb_msg.add_task_torso_ori_vel(data->task_torso_ori_vel[i]);
    pb_msg.add_task_torso_ori_pos_des(data->task_torso_ori_pos_des[i]);
    pb_msg.add_task_torso_ori_vel_des(data->task_torso_ori_vel_des[i]);
    pb_msg.add_task_torso_ori_acc_des(data->task_torso_ori_acc_des[i]);

    pb_msg.add_task_torso_ori_pos_local(data->task_torso_ori_pos_local[i]);
    pb_msg.add_task_torso_ori_vel_local(data->task_torso_ori_vel_local[i]);
    pb_msg.add_task_torso_ori_pos_des_local(
        data->task_torso_ori_pos_des_local[i]);
    pb_msg.add_task_torso_ori_vel_des_local(
        data->task_torso_ori_vel_des_local[i]);
    pb_msg.add_task_torso_ori_acc_des_local(
        data->task_torso_ori_acc_des_local[i]);

    pb_msg.add_task_rfoot_lin_pos(data->task_rfoot_lin_pos[i]);
    pb_msg.add_task_rfoot_lin_vel(data->task_rfoot_lin_vel[i]);
    pb_msg.add_task_rfoot_lin_pos_des(data->task_rfoot_lin_pos_des[i]);
    pb_msg.add_task_rfoot_lin_vel_des(data->task_rfoot_lin_vel_des[i]);
    pb_msg.add_task_rfoot_lin_acc_des(data->task_rfoot_lin_acc_des[i]);

    pb_msg.add_task_rfoot_lin_pos_local(data->task_rfoot_lin_pos_local[i]);
    pb_msg.add_task_rfoot_lin_vel_local(data->task_rfoot_lin_vel_local[i]);
    pb_msg.add_task_rfoot_lin_pos_des_local(
        data->task_rfoot_lin_pos_des_local[i]);
    pb_msg.add_task_rfoot_lin_vel_des_local(
        data->task_rfoot_lin_vel_des_local[i]);
    pb_msg.add_task_rfoot_lin_acc_des_local(
        data->task_rfoot_lin_acc_des_local[i]);

    pb_msg.add_task_rfoot_ori_pos(data->task_rfoot_ori_pos[i]);
    pb_msg.add_task_rfoot_ori_vel(data->task_rfoot_ori_vel[i]);
    pb_msg.add_task_rfoot_ori_pos_des(data->task_rfoot_ori_pos_des[i]);
    pb_msg.add_task_rfoot_ori_vel_des(data->task_rfoot_ori_vel_des[i]);
    pb_msg.add_task_rfoot_ori_acc_des(data->task_rfoot_ori_acc_des[i]);

    pb_msg.add_task_rfoot_ori_pos_local(data->task_rfoot_ori_pos_local[i]);
    pb_msg.add_task_rfoot_ori_vel_local(data->task_rfoot_ori_vel_local[i]);
    pb_msg.add_task_rfoot_ori_pos_des_local(
        data->task_rfoot_ori_pos_des_local[i]);
    pb_msg.add_task_rfoot_ori_vel_des_local(
        data->task_rfoot_ori_vel_des_local[i]);
    pb_msg.add_task_rfoot_ori_acc_des_local(
        data->task_rfoot_ori_acc_des_local[i]);

    pb_msg.add_task_lfoot_lin_pos(data->task_lfoot_lin_pos[i]);
    pb_msg.add_task_lfoot_lin_vel(data->task_lfoot_lin_vel[i]);
    pb_msg.add_task_lfoot_lin_pos_des(data->task_lfoot_lin_pos_des[i]);
    pb_msg.add_task_lfoot_lin_vel_des(data->task_lfoot_lin_vel_des[i]);
    pb_msg.add_task_lfoot_lin_acc_des(data->task_lfoot_lin_acc_des[i]);

    pb_msg.add_task_lfoot_lin_pos_local(data->task_lfoot_lin_pos_local[i]);
    pb_msg.add_task_lfoot_lin_vel_local(data->task_lfoot_lin_vel_local[i]);
    pb_msg.add_task_lfoot_lin_pos_des_local(
        data->task_lfoot_lin_pos_des_local[i]);
    pb_msg.add_task_lfoot_lin_vel_des_local(
        data->task_lfoot_lin_vel_des_local[i]);
    pb_msg.add_task_lfoot_lin_acc_des_local(
        data->task_lfoot_lin_acc_des_local[i]);

    pb_msg.add_task_lfoot_ori_pos(data->task_lfoot_ori_pos[i]);
    pb_msg.add_task_lfoot_ori_vel(data->task_lfoot_ori_vel[i]);
    pb_msg.add_task_lfoot_ori_pos_des(data->task_lfoot_ori_pos_des[i]);
    pb_msg.add_task_lfoot_ori_vel_des(data->task_lfoot_ori_vel_des[i]);
    pb_msg.add_task_lfoot_ori_acc_des(data->task_lfoot_ori_acc_des[i]);

    pb_msg.add_task_lfoot_ori_pos_local(data->task_lfoot_ori_pos_local[i]);
    pb_msg.add_task_lfoot_ori_vel_local(data->task_lfoot_ori_vel_local[i]);
    pb_msg.add_task_lfoot_ori_pos_des_local(
        data->task_lfoot_ori_pos_des_local[i]);
    pb_msg.add_task_lfoot_ori_vel_des_local(
        data->task_lfoot_ori_vel_des_local[i]);
    pb_msg.add_task_lfoot_ori_acc_des_local(
        data->task_lfoot_ori_acc_des_local[i]);
  }

  pb_msg.add_task_torso_ori_pos_des(data->task_torso_ori_pos_des[3]);
  pb_msg.add_task_torso_ori_pos(data->task_torso_ori_pos[3]);
  pb_msg.add_task_torso_ori_pos_des_local(
      data->task_torso_ori_pos_des_local[3]);
  pb_msg.add_task_torso_ori_pos_local(data->task_torso_ori_pos_local[3]);

  pb_msg.add_task_lfoot_ori_pos_des(data->task_lfoot_ori_pos_des[3]);
  pb_msg.add_task_lfoot_ori_pos(data->task_lfoot_ori_pos[3]);
  pb_msg.add_task_lfoot_ori_pos_des_local(
      data->task_lfoot_ori_pos_des_local[3]);
  pb_msg.add_task_lfoot_ori_pos_local(data->task_lfoot_ori_pos_local[3]);

  pb_msg.add_task_rfoot_ori_pos_des(data->task_rfoot_ori_pos_des[3]);
  pb_msg.add_task_rfoot_ori_pos(data->task_rfoot_ori_pos[3]);
  pb_msg.add_task_rfoot_ori_pos_des_local(
      data->task_rfoot_ori_pos_des_local[3]);
  pb_msg.add_task_rfoot_ori_pos_local(data->task_rfoot_ori_pos_local[3]);

  for (int i = 0; i < data->task_upper_body_pos_des.size(); ++i) {
    pb_msg.add_task_upper_body_pos_des(data->task_upper_body_pos_des[i]);
    pb_msg.add_task_upper_body_vel_des(data->task_upper_body_vel_des[i]);
    pb_msg.add_task_upper_body_acc_des(data->task_upper_body_acc_des[i]);
    pb_msg.add_task_upper_body_pos(data->task_upper_body_pos[i]);
    pb_msg.add_task_upper_body_vel(data->task_upper_body_vel[i]);
  }

  for (int i = 0; i < 6; ++i) {
    pb_msg.add_cmd_rfoot_rf(data->cmd_rfoot_rf[i]);
    pb_msg.add_cmd_lfoot_rf(data->cmd_lfoot_rf[i]);
  }

  for (int i = 0; i < data->joint_positions.size(); ++i) {
    pb_msg.add_joint_positions(data->joint_positions[i]);
    pb_msg.add_joint_velocities(data->joint_velocities[i]);
  }

  for (int i = 0; i < 3; ++i) {
    pb_msg.add_base_joint_pos(data->base_joint_pos[i]);
    pb_msg.add_base_joint_quat(data->base_joint_quat[i]);
  }
  pb_msg.add_base_joint_quat(data->base_joint_quat[3]);

  for (int i = 0; i < data->cmd_joint_positions.size(); ++i) {
    pb_msg.add_cmd_joint_positions(data->cmd_joint_positions[i]);
    pb_msg.add_cmd_joint_velocities(data->cmd_joint_velocities[i]);
    pb_msg.add_cmd_joint_torques(data->cmd_joint_torques[i]);
  }

  for (int i = 0; i < 3; ++i) {
    pb_msg.add_com_vel_est(data->com_vel_est[i]);
    pb_msg.add_com_vel_raw(data->com_vel_raw[i]);
    pb_msg.add_imu_ang_vel_est(data->imu_ang_vel_est[i]);
    pb_msg.add_imu_ang_vel_raw(data->imu_ang_vel_raw[i]);
    pb_msg.add_cam_est(data->cam_est[i]);
    pb_msg.add_cam_raw(data->cam_raw[i]);
  }

  for (int i = 0; i < 2; ++i) {
    pb_msg.add_icp(data->icp[i]);
    pb_msg.add_icp_des(data->icp_des[i]);
    pb_msg.add_icp_dot(data->icp_dot[i]);
    pb_msg.add_icp_dot_des(data->icp_dot_des[i]);
  }

  // serialize
  std::string serialized_str;
  pb_msg.SerializeToString(&serialized_str);

  // send
  zmq::message_t zmq_msg(serialized_str.size());
  memcpy((void *)zmq_msg.data(), serialized_str.c_str(), serialized_str.size());
  socket_->send(zmq_msg);
}
