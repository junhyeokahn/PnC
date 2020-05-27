#include <PnC/ValkyriePnC/StateMachines/DoubleSupportBalance.hpp>
#include <PnC/ValkyriePnC/CtrlArchitectures/ValkyrieControlArchitecture.hpp>

DoubleSupportBalance::DoubleSupportBalance(const StateIdentifier state_identifier_in, ValkyrieControlArchitecture* _ctrl_arch, RobotSystem* _robot) : 
               StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Double Support Balance");

  // Set Trigger to false
  state_switch_button_trigger_ = false;

  // Set Pointer to Control Architecture
  val_ctrl_arch_ = ((ValkyrieControlArchitecture*) _ctrl_arch);
  taf_container_ = val_ctrl_arch_->taf_container_;
  // Get State Provider
  sp_ = ValkyrieStateProvider::getStateProvider(robot_);

}

DoubleSupportBalance::~DoubleSupportBalance(){
}


void DoubleSupportBalance::firstVisit(){
  // Reset Flags
  state_switch_button_trigger_ = false;

  std::cout << "Double Support Balance First Visit" << std::endl;
  ctrl_start_time_ = sp_->curr_time;
}

void DoubleSupportBalance::_taskUpdate(){
  // =========================================================================
  // Set Foot Motion Tasks
  // =========================================================================
  val_ctrl_arch_->rfoot_trajectory_manager_->useCurrent();
  val_ctrl_arch_->lfoot_trajectory_manager_->useCurrent();
}

void DoubleSupportBalance::oneStep(){  
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  _taskUpdate();
}

void DoubleSupportBalance::lastVisit(){  
}

bool DoubleSupportBalance::endOfState(){  
  if (state_switch_button_trigger_){
    std::cout << "[DoubleSupportBalance] Switch State Triggered" << std::endl;
    return true;
  }
  return false;
} 

StateIdentifier DoubleSupportBalance::getNextState(){
  return VALKYRIE_STATES::INITIAL_TRANSFER;
}


void DoubleSupportBalance::initialization(const YAML::Node& node){
}
