#include <math.h>
#include <stdio.h>
#include <string>

#include <PnC/AtlasPnC/AtlasControlArchitecture.hpp>
#include <PnC/AtlasPnC/AtlasInterface.hpp>
#include <PnC/AtlasPnC/AtlasInterruptLogic.hpp>
#include <PnC/AtlasPnC/AtlasStateEstimator.hpp>
#include <PnC/AtlasPnC/AtlasStateProvider.hpp>
#include <PnC/RobotSystem/DartRobotSystem.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

AtlasInterface::AtlasInterface() : Interface() {
  std::string border = "=";
  for (int i = 0; i < 79; ++i) {
    border += "=";
  }
  myUtils::color_print(myColor::BoldCyan, border);
  myUtils::pretty_constructor(0, "Atlas Interface");

  YAML::Node cfg = YAML::LoadFile(THIS_COM "Config/Atlas/pnc.yaml");

  robot_ = new DartRobotSystem(THIS_COM "RobotModel/atlas/atlas_rel_path.urdf",
                               false, false);
  se_ = new AtlasStateEstimator(robot_);
  sp_ = AtlasStateProvider::getStateProvider(robot_);
  sp_->servo_rate = myUtils::readParameter<double>(cfg, "servo_rate");

  count_ = 0;
  waiting_count_ = 2;

  control_architecture_ = new AtlasControlArchitecture(robot_);
  interrupt = new AtlasInterruptLogic(
      static_cast<AtlasControlArchitecture *>(control_architecture_));

  myUtils::color_print(myColor::BoldCyan, border);
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

  std::cout << "count_: " << count_ << std::endl;
  // if (count_ <= waiting_count_) {
  /*se_->initialize(data);*/
  // SetSafeCommand(data, cmd);
  //} else {
  // std::cout << "1" << std::endl;
  // se_->update(data);
  // std::cout << "2" << std::endl;
  // interrupt->processInterrupts();
  // std::cout << "3" << std::endl;
  // control_architecture_->getCommand(cmd);
  // std::cout << "4" << std::endl;
  /*}*/

  if (count_ == 0) {
    se_->initialize(data);
  }
  std::cout << "1" << std::endl;
  se_->update(data);
  std::cout << "2" << std::endl;
  interrupt->processInterrupts();
  std::cout << "3" << std::endl;
  control_architecture_->getCommand(cmd);
  std::cout << "4" << std::endl;

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
