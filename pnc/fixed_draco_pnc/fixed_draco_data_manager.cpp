#include "pnc/fixed_draco_pnc/fixed_draco_data_manager.hpp"

FixedDracoDataManager::FixedDracoDataManager() {
  b_initialized_ = false;
  data = std::make_unique<FixedDracoData>();
}

FixedDracoDataManager::~FixedDracoDataManager() {}

FixedDracoDataManager *FixedDracoDataManager::GetFixedDracoDataManager() {
  static FixedDracoDataManager instance;
  return &instance;
}

void FixedDracoDataManager::InitializeSockets(const std::string &_addr) {
  context_ = std::make_unique<zmq::context_t>(1);
  socket_ = std::make_unique<zmq::socket_t>(*context_, ZMQ_PUB);
  socket_->bind(_addr);
  b_initialized_ = true;
}

void FixedDracoDataManager::Send() {
  assert(b_initialized_);

  // copy data to protobuf msg
  fixed_draco::pnc_msg pb_msg;

  pb_msg.set_time(data->time);
  pb_msg.set_phase(data->phase);

  for (int i = 0; i < 3; ++i) {
    pb_msg.add_task_rfoot_pos(data->task_rfoot_pos[i]);
    pb_msg.add_task_rfoot_vel(data->task_rfoot_vel[i]);
    pb_msg.add_task_rfoot_pos_des(data->task_rfoot_pos_des[i]);
    pb_msg.add_task_rfoot_vel_des(data->task_rfoot_vel_des[i]);
    pb_msg.add_task_rfoot_acc_des(data->task_rfoot_acc_des[i]);

    pb_msg.add_task_rfoot_ori(data->task_rfoot_ori[i]);
    pb_msg.add_task_rfoot_ang_vel(data->task_rfoot_ang_vel[i]);
    pb_msg.add_task_rfoot_ori_des(data->task_rfoot_ori_des[i]);
    pb_msg.add_task_rfoot_ang_vel_des(data->task_rfoot_ang_vel_des[i]);
    pb_msg.add_task_rfoot_ang_acc_des(data->task_rfoot_ang_acc_des[i]);

    pb_msg.add_task_lfoot_pos(data->task_lfoot_pos[i]);
    pb_msg.add_task_lfoot_vel(data->task_lfoot_vel[i]);
    pb_msg.add_task_lfoot_pos_des(data->task_lfoot_pos_des[i]);
    pb_msg.add_task_lfoot_vel_des(data->task_lfoot_vel_des[i]);
    pb_msg.add_task_lfoot_acc_des(data->task_lfoot_acc_des[i]);

    pb_msg.add_task_lfoot_ori(data->task_lfoot_ori[i]);
    pb_msg.add_task_lfoot_ang_vel(data->task_lfoot_ang_vel[i]);
    pb_msg.add_task_lfoot_ori_des(data->task_lfoot_ori_des[i]);
    pb_msg.add_task_lfoot_ang_vel_des(data->task_lfoot_ang_vel_des[i]);
    pb_msg.add_task_lfoot_ang_acc_des(data->task_lfoot_ang_acc_des[i]);
  }
  pb_msg.add_task_lfoot_ori_des(data->task_lfoot_ori_des[3]);
  pb_msg.add_task_lfoot_ori(data->task_lfoot_ori[3]);

  pb_msg.add_task_rfoot_ori_des(data->task_rfoot_ori_des[3]);
  pb_msg.add_task_rfoot_ori(data->task_rfoot_ori[3]);

  for (int i = 0; i < data->task_upper_body_pos_des.size(); ++i) {
    pb_msg.add_task_upper_body_pos_des(data->task_upper_body_pos_des[i]);
    pb_msg.add_task_upper_body_vel_des(data->task_upper_body_vel_des[i]);
    pb_msg.add_task_upper_body_acc_des(data->task_upper_body_acc_des[i]);
    pb_msg.add_task_upper_body_pos(data->task_upper_body_pos[i]);
    pb_msg.add_task_upper_body_vel(data->task_upper_body_vel[i]);
  }

  for (int i = 0; i < data->joint_positions.size(); ++i) {
    pb_msg.add_joint_positions(data->joint_positions[i]);
    pb_msg.add_joint_velocities(data->joint_velocities[i]);
  }

  for (int i = 0; i < data->cmd_joint_positions.size(); ++i) {
    pb_msg.add_cmd_joint_positions(data->cmd_joint_positions[i]);
    pb_msg.add_cmd_joint_velocities(data->cmd_joint_velocities[i]);
    pb_msg.add_cmd_joint_torques(data->cmd_joint_torques[i]);
  }

  // serialize
  std::string serialized_str;
  pb_msg.SerializeToString(&serialized_str);

  // send
  zmq::message_t zmq_msg(serialized_str.size());
  memcpy((void *)zmq_msg.data(), serialized_str.c_str(), serialized_str.size());
  socket_->send(zmq_msg);
}
