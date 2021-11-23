#pragma once

#include <map>
#include <string>

#include "pnc/draco_pnc/draco_data_manager.hpp"
#include "pnc/interface.hpp"

class DracoStateProvider;
class DracoStateEstimator;

class DracoSensorData {
public:
  DracoSensorData() {
    imu_frame_iso.setIdentity();
    imu_frame_vel.setZero();
    b_rf_contact = false;
    b_lf_contact = false;
  }
  virtual ~DracoSensorData() {}

  Eigen::Matrix<double, 4, 4> imu_frame_iso;
  Eigen::Matrix<double, 6, 1> imu_frame_vel;
  std::map<std::string, double> joint_positions;
  std::map<std::string, double> joint_velocities;

  bool b_rf_contact;
  bool b_lf_contact;

  // for debugging purpose
  Eigen::Vector3d base_com_pos;
  Eigen::Vector4d base_com_quat;
  Eigen::Vector3d base_com_lin_vel;
  Eigen::Vector3d base_com_ang_vel;

  Eigen::Vector3d base_joint_pos;
  Eigen::Vector4d base_joint_quat;
  Eigen::Vector3d base_joint_lin_vel;
  Eigen::Vector3d base_joint_ang_vel;
};

class DracoCommand {
public:
  DracoCommand() {}
  virtual ~DracoCommand() {}

  std::map<std::string, double> joint_positions;
  std::map<std::string, double> joint_velocities;
  std::map<std::string, double> joint_torques;

  double l_knee_int_frc = 0.;
  double r_knee_int_frc = 0.;
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
