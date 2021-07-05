#include <math.h>
#include <stdio.h>
#include <string>

#include "utils/util.hpp"
#include <PnC/AtlasPnC/AtlasControlArchitecture.hpp>
#include <PnC/AtlasPnC/AtlasInterface.hpp>
#include <PnC/AtlasPnC/AtlasInterruptLogic.hpp>
#include <PnC/AtlasPnC/AtlasStateEstimator.hpp>
#include <PnC/AtlasPnC/AtlasStateProvider.hpp>
#include <PnC/RobotSystem/DartRobotSystem.hpp>

AtlasInterface::AtlasInterface() : Interface() {
  std::string border = "=";
  for (int i = 0; i < 79; ++i) {
    border += "=";
  }
  util::ColorPrint(color::kBoldCyan, border);
  util::PrettyConstructor(0, "Atlas Interface");

  YAML::Node cfg = YAML::LoadFile(THIS_COM "Config/Atlas/pnc.yaml");

  robot_ = new DartRobotSystem(THIS_COM "RobotModel/atlas/atlas_rel_path.urdf",
                               false, false);
  se_ = new AtlasStateEstimator(robot_);
  sp_ = AtlasStateProvider::getStateProvider(robot_);
  sp_->servo_rate = util::ReadParameter<double>(cfg, "servo_rate");

  count_ = 0;
  waiting_count_ = 2;

  control_architecture_ = new AtlasControlArchitecture(robot_);
  interrupt = new AtlasInterruptLogic(
      static_cast<AtlasControlArchitecture *>(control_architecture_));

  util::ColorPrint(color::kBoldCyan, border);
}

AtlasInterface::~AtlasInterface() {
  delete robot_;
  delete se_;
  delete interrupt;
  delete control_architecture_;
}

void AtlasInterface::getCommand(void *_data, void *_command) {
  AtlasCommand *cmd = ((AtlasCommand *)_command);
  AtlasSensorData *data = ((AtlasSensorData *)_data);

  if (count_ == 0) {
    se_->initialize(data);
    // DataManager::GetDataManager()->start();
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

void AtlasInterface::SetSafeCommand(AtlasSensorData *data, AtlasCommand *cmd) {
  for (std::map<std::string, double>::iterator it =
           data->joint_positions.begin();
       it != data->joint_positions.end(); it++) {
    cmd->joint_positions[it->first] = data->joint_positions[it->first];
    cmd->joint_velocities[it->first] = 0.;
    cmd->joint_torques[it->first] = 0.;
  }
}
