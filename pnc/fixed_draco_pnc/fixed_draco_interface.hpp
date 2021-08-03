#pragma once

#include <map>
#include <string>

#include "pnc/fixed_draco_pnc/fixed_draco_data_manager.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_estimator.hpp"
#include "pnc/interface.hpp"

class FixedDracoStateProvider;

class FixedDracoSensorData {
public:
  FixedDracoSensorData() {
    imu_frame_iso.setIdentity();
    imu_frame_vel.setZero();
  }
  virtual ~FixedDracoSensorData() {}

  Eigen::Matrix<double, 4, 4> imu_frame_iso;
  Eigen::Matrix<double, 6, 1> imu_frame_vel;
  std::map<std::string, double> joint_positions;
  std::map<std::string, double> joint_velocities;
};

class FixedDracoCommand {
public:
  FixedDracoCommand() {}
  virtual ~FixedDracoCommand() {}

  std::map<std::string, double> joint_positions;
  std::map<std::string, double> joint_velocities;
  std::map<std::string, double> joint_torques;
};

class FixedDracoInterface : public Interface {
protected:
  FixedDracoStateProvider *sp_;
  FixedDracoStateEstimator *se_;

  int waiting_count_;

  void SetSafeCommand(FixedDracoSensorData *data, FixedDracoCommand *cmd);

public:
  FixedDracoInterface(bool b_sim);
  virtual ~FixedDracoInterface();
  virtual void getCommand(void *_sensor_data, void *_command_data);
};
