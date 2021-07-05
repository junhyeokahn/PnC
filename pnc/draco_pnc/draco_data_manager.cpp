#include "pnc/draco_pnc/draco_data_manager.hpp"

DracoDataManager::DracoDataManager() {
  b_initialized_ = false;
  data = std::make_unique<DracoData>();
}

DracoDataManager::~DracoDataManager() {}

DracoDataManager *DracoDataManager::GetDracoDataManager() {
  static DracoDataManager instance;
  return &instance;
}

void DracoDataManager::InitializeSockets(const std::string &_addr) {
  context_ = std::make_unique<zmq::context_t>(1);
  socket_ = std::make_unique<zmq::socket_t>(*context_, ZMQ_PUB);
  socket_->bind(_addr);
  b_initialized_ = true;
}

void DracoDataManager::Send() {
  assert(b_initialized_);

  // copy data to protobuf msg
  draco::pnc_msg pb_msg;

  pb_msg.set_time(data->time);
  pb_msg.set_phase(data->phase);

  for (int i = 0; i < 3; ++i) {
    pb_msg.add_task_com_pos(data->task_com_pos[i]);
    pb_msg.add_task_com_vel(data->task_com_vel[i]);
    pb_msg.add_task_com_pos_des(data->task_com_pos_des[i]);
    pb_msg.add_task_com_vel_des(data->task_com_vel_des[i]);
    pb_msg.add_task_com_acc_des(data->task_com_acc_des[i]);
  }

  // serialize
  std::string serialized_str;
  pb_msg.SerializeToString(&serialized_str);

  // send
  zmq::message_t zmq_msg(serialized_str.size());
  memcpy((void *)zmq_msg.data(), serialized_str.c_str(), serialized_str.size());
  socket_->send(zmq_msg);
}
