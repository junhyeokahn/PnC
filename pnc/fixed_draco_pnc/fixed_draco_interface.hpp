#pragma once

#include <map>
#include <string>

#include "pnc/fixed_draco_pnc/fixed_draco_data_manager.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_estimator.hpp"
#include "pnc/interface.hpp"

class FixedDracoStateProvider;

class FixedDracoSensorData {
public:
  FixedDracoSensorData() {}
  virtual ~FixedDracoSensorData() {}

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
  int smoothing_count_;

  void SetSafeCommand(FixedDracoSensorData *data, FixedDracoCommand *cmd);
  void SmoothCommand(FixedDracoCommand *cmd);

public:
  FixedDracoInterface(bool b_sim);
  virtual ~FixedDracoInterface();
  virtual void getCommand(void *_sensor_data, void *_command_data);
};
