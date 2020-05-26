#include <PnC/ValkyriePnC/StateMachines/InitialTransfer.hpp>
#include <PnC/ValkyriePnC/CtrlArchitectures/ValkyrieControlArchitecture.hpp>

InitialTransfer::InitialTransfer(const StateIdentifier state_identifier_in, ValkyrieControlArchitecture* _ctrl_arch, RobotSystem* _robot) : 
               StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Double Support Stand");

  // Set Pointer to Control Architecture
  val_ctrl_arch_ = ((ValkyrieControlArchitecture*) _ctrl_arch);
  taf_container_ = val_ctrl_arch_->taf_container_;
  // Get State Provider
  sp_ = ValkyrieStateProvider::getStateProvider(robot_);

}

InitialTransfer::~InitialTransfer(){
}


void InitialTransfer::firstVisit(){
  ctrl_start_time_ = sp_->curr_time;
}

void InitialTransfer::_taskUpdate(){
}

void InitialTransfer::oneStep(){  
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  _taskUpdate();
}

void InitialTransfer::lastVisit(){  
}

bool InitialTransfer::endOfState(){  
  return false;
} 

StateIdentifier InitialTransfer::getNextState(){
}


void InitialTransfer::initialization(const YAML::Node& node){
}
