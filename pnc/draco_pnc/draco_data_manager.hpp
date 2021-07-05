#pragma once

#include <Eigen/Dense>
#include <zmq.hpp>

#include "build/messages/draco.pb.h"

class DracoData {
public:
  DracoData() {
    time = 0.;
    phase = 0;
    task_com_pos.setZero();
    task_com_vel.setZero();
    task_com_pos_des.setZero();
    task_com_vel_des.setZero();
    task_com_acc_des.setZero();
  };
  ~DracoData(){};

  // should be matching with protobuf msg
  double time;
  int phase;
  Eigen::Vector3d task_com_pos;
  Eigen::Vector3d task_com_vel;
  Eigen::Vector3d task_com_pos_des;
  Eigen::Vector3d task_com_vel_des;
  Eigen::Vector3d task_com_acc_des;
};

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

  bool b_initialized_;
};
