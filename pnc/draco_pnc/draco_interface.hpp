#pragma once

#include <map>
#include <string>

#include "pnc/draco_pnc/draco_data_manager.hpp"
#include "pnc/interface.hpp"

class DracoStateProvider;
class DracoStateEstimator;

class DracoSensorData {
public:
  DracoSensorData() {}
  virtual ~DracoSensorData() {
    imu_frame_iso.setZero();
    imu_frame_vel.setZero();
  }

  Eigen::Matrix<double, 4, 4> imu_frame_iso;
  Eigen::Matrix<double, 6, 1> imu_frame_vel;
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
  DracoInterface(bool b_sim);
  virtual ~DracoInterface();
  virtual void getCommand(void *_sensor_data, void *_command_data);
};
