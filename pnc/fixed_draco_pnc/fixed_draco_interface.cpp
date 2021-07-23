#include "pnc/fixed_draco_pnc/fixed_draco_interface.hpp"

#include <math.h>
#include <stdio.h>
#include <string>

#include "pnc/fixed_draco_pnc/fixed_draco_control_architecture.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_interrupt_logic.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_provider.hpp"
#include "pnc/robot_system/dart_robot_system.hpp"
#include "utils/util.hpp"

FixedDracoInterface::FixedDracoInterface(bool _b_sim) : Interface() {
  std::string border = "=";
  for (int i = 0; i < 79; ++i) {
    border += "=";
  }
  util::ColorPrint(color::kBoldCyan, border);
  util::PrettyConstructor(0, "Draco Interface");

  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/fixed_draco/pnc.yaml");

  robot_ = new DartRobotSystem(THIS_COM "robot_model/draco/draco_rel_path.urdf",
                               true, false);
  se_ = new FixedDracoStateEstimator(robot_);
  sp_ = FixedDracoStateProvider::getStateProvider();
  sp_->servo_dt = util::ReadParameter<double>(cfg, "servo_dt");
  sp_->save_freq = util::ReadParameter<int>(cfg, "save_freq");
  sp_->smoothing_duration =
      util::ReadParameter<double>(cfg, "smoothing_duration");

  count_ = 0;
  waiting_count_ = 10;

  control_architecture_ = new FixedDracoControlArchitecture(robot_);
  if (_b_sim) {
    control_architecture_->state = fixed_draco_states::kHold;
    sp_->state = control_architecture_->state;
  } else {
    control_architecture_->state = fixed_draco_states::kInitialize;
    sp_->state = control_architecture_->state;
  }
  interrupt = new FixedDracoInterruptLogic(
      static_cast<FixedDracoControlArchitecture *>(control_architecture_));

  FixedDracoDataManager::GetFixedDracoDataManager()->InitializeSockets(
      util::ReadParameter<std::string>(cfg, "ip_addr"));

  util::ColorPrint(color::kBoldCyan, border);
}

FixedDracoInterface::~FixedDracoInterface() {
  delete robot_;
  delete se_;
  delete interrupt;
  delete control_architecture_;
}

void FixedDracoInterface::getCommand(void *_data, void *_command) {
  running_time_ = (double)(count_)*sp_->servo_dt;
  sp_->count = count_;
  sp_->curr_time = running_time_;
  sp_->state = control_architecture_->state;
  sp_->prev_state = control_architecture_->prev_state;

  FixedDracoCommand *cmd = ((FixedDracoCommand *)_command);
  FixedDracoSensorData *data = ((FixedDracoSensorData *)_data);

  if (count_ <= waiting_count_) {
    se_->initialize(data);
    this->SetSafeCommand(data, cmd);
  } else {
    se_->update(data);
    interrupt->processInterrupts();
    control_architecture_->getCommand(cmd);
    if (sp_->b_smoothing_cmd) {
      this->SmoothCommand(cmd);
    }
  }

  if (sp_->count % sp_->save_freq == 0) {
    FixedDracoDataManager::GetFixedDracoDataManager()->data->time =
        sp_->curr_time;
    FixedDracoDataManager::GetFixedDracoDataManager()->data->phase = sp_->state;
    FixedDracoDataManager::GetFixedDracoDataManager()->Send();
  }

  ++count_;
}

void FixedDracoInterface::SmoothCommand(FixedDracoCommand *cmd) {
  double s = util::SmoothPos(0., 1., sp_->smoothing_duration,
                             sp_->curr_time - sp_->smoothing_start_time);
  for (std::map<std::string, double>::iterator it =
           cmd->joint_velocities.begin();
       it != cmd->joint_velocities.end(); ++it) {
    cmd->joint_velocities[it->first] *= s;
    cmd->joint_torques[it->first] *= s;
  }
  if (sp_->curr_time >= sp_->smoothing_start_time + sp_->smoothing_duration) {
    sp_->b_smoothing_cmd = false;
  }
}

void FixedDracoInterface::SetSafeCommand(FixedDracoSensorData *data,
                                         FixedDracoCommand *cmd) {
  for (std::map<std::string, double>::iterator it =
           data->joint_positions.begin();
       it != data->joint_positions.end(); it++) {
    cmd->joint_positions[it->first] = data->joint_positions[it->first];
    cmd->joint_velocities[it->first] = 0.;
    cmd->joint_torques[it->first] = 0.;
  }
}
