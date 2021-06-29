#include "PnC/draco_pnc/draco_interface.hpp"

#include <math.h>
#include <stdio.h>
#include <string>

#include "PnC/RobotSystem/DartRobotSystem.hpp"
#include "PnC/draco_pnc/draco_control_architecture.hpp"
#include "PnC/draco_pnc/draco_interrupt_logic.hpp"
#include "PnC/draco_pnc/draco_state_estimator.hpp"
#include "PnC/draco_pnc/draco_state_provider.hpp"
#include "Utils/IO/IOUtilities.hpp"
#include "Utils/Math/MathUtilities.hpp"

DracoInterface::DracoInterface() : Interface() {
  std::string border = "=";
  for (int i = 0; i < 79; ++i) {
    border += "=";
  }
  myUtils::color_print(myColor::BoldCyan, border);
  myUtils::pretty_constructor(0, "Draco Interface");

  YAML::Node cfg = YAML::LoadFile(THIS_COM "Config/draco/pnc.yaml");

  robot_ = new DartRobotSystem(THIS_COM "RobotModel/draco/draco_rel_path.urdf",
                               false, false);
  se_ = new DracoStateEstimator(robot_);
  sp_ = DracoStateProvider::getStateProvider();
  sp_->servo_rate = myUtils::readParameter<double>(cfg, "servo_rate");

  count_ = 0;
  waiting_count_ = 2;

  control_architecture_ = new DracoControlArchitecture(robot_);
  interrupt = new DracoInterruptLogic(
      static_cast<DracoControlArchitecture *>(control_architecture_));

  myUtils::color_print(myColor::BoldCyan, border);
}

DracoInterface::~DracoInterface() {
  delete robot_;
  delete se_;
  delete interrupt;
  delete control_architecture_;
}

void DracoInterface::getCommand(void *_data, void *_command) {
  DracoCommand *cmd = ((DracoCommand *)_command);
  DracoSensorData *data = ((DracoSensorData *)_data);

  if (count_ == 0) {
    se_->initialize(data);
    DataManager::GetDataManager()->start();
  }
  se_->update(data);
  interrupt->processInterrupts();
  control_architecture_->getCommand(cmd);

  ++count_;
  running_time_ = (double)(count_)*sp_->servo_rate;
  sp_->curr_time = running_time_;
  sp_->prev_state = control_architecture_->prev_state;
  sp_->state = control_architecture_->state;
}

void DracoInterface::SetSafeCommand(DracoSensorData *data, DracoCommand *cmd) {
  for (std::map<std::string, double>::iterator it =
           data->joint_positions.begin();
       it != data->joint_positions.end(); it++) {
    cmd->joint_positions[it->first] = data->joint_positions[it->first];
    cmd->joint_velocities[it->first] = 0.;
    cmd->joint_torques[it->first] = 0.;
  }
}
