#include <math.h>
#include <stdio.h>
#include <string>

//#include <PnC/AtlasPnC/AtlasCtrlArchitecture/AtlasControlArchitecture.hpp>
#include <PnC/AtlasPnC/AtlasInterface.hpp>
//#include <PnC/AtlasPnC/AtlasLogicInterrupt/WalkingInterruptLogic.hpp>
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

  robot_ = new DartRobotSystem(
      THIS_COM "RobotModel/Robot/atlas/atlas_rel_path.urdf", false, true);
  se_ = new AtlasStateEstimator(robot_);
  sp_ = AtlasStateProvider::getStateProvider(robot_);

  count_ = 0;
  waiting_count_ = 2;

  // control_architecture_ = new AtlasControlArchitecture(robot_);
  // interrupt = new WalkingInterruptLogic(
  // static_cast<ValkyrieControlArchitecture *>(control_architecture_));

  myUtils::color_print(myColor::BoldCyan, border);
}

AtlasInterface::~AtlasInterface() {
  delete robot_;
  delete se_;
  // delete interrupt;
  // delete control_architecture_;
}

void AtlasInterface::getCommand(void *_data, void *_command) {
  AtlasCommand *cmd = ((AtlasCommand *)_command);
  AtlasSensorData *data = ((AtlasSensorData *)_data);

  if (count_ <= waiting_count_) {
    se_->initialize(data);
  } else {
    se_->update(data);
    // interrupt->processInterrupts();
    // control_architecture_->getCommand(cmd);
  }

  ++count_;
  running_time_ = (double)(count_)*sp_->dt;
  sp_->curr_time = running_time_;
  sp_->prev_state = sp_->state;
  // sp_->state = control_architecture_->getState();
}
