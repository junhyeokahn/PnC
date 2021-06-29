#pragma once

#include <map>
#include <string>

#include "PnC/Interface.hpp"

class DracoStateProvider;
class DracoStateEstimator;

class DracoSensorData {
public:
  DracoSensorData() {}
  virtual ~DracoSensorData() {
    base_com_pos.setZero();
    base_com_quat.setZero();
    base_com_lin_vel.setZero();
    base_com_ang_vel.setZero();
    base_joint_pos.setZero();
    base_joint_quat.setZero();
    base_joint_lin_vel.setZero();
    base_joint_ang_vel.setZero();
  }

  Eigen::Vector3d base_com_pos;
  Eigen::Vector4d base_com_quat; // scalar first quaternion (w, x, y, z)
  Eigen::Vector3d base_com_lin_vel;
  Eigen::Vector3d base_com_ang_vel;
  Eigen::Vector3d base_joint_pos;
  Eigen::Vector4d base_joint_quat; // scalar first quaternion (w, x, y, z)
  Eigen::Vector3d base_joint_lin_vel;
  Eigen::Vector3d base_joint_ang_vel;
  std::map<std::string, double> joint_positions;
  std::map<std::string, double> joint_velocities;

  bool b_rf_contact;
  bool b_lf_contact;
};

class DracoCommand {
public:
  DracoCommand() {}
  virtual ~DracoCommand() {}

  std::map<std::string, double> joint_positions;
  std::map<std::string, double> joint_velocities;
  std::map<std::string, double> joint_torques;
};

class DracoInterface : public Interface {
protected:
  DracoStateEstimator *se_;
  DracoStateProvider *sp_;

  int waiting_count_;

  void SetSafeCommand(DracoSensorData *data, DracoCommand *cmd);

public:
  DracoInterface();
  virtual ~DracoInterface();
  virtual void getCommand(void *_sensor_data, void *_command_data);
};
