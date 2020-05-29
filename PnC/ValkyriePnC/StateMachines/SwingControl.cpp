#include <PnC/ValkyriePnC/StateMachines/SwingControl.hpp>
#include <PnC/ValkyriePnC/CtrlArchitectures/ValkyrieControlArchitecture.hpp>

SwingControl::SwingControl(const StateIdentifier state_identifier_in, 
                                     const int _leg_side, 
                                     ValkyrieControlArchitecture* _ctrl_arch, RobotSystem* _robot) : 
               StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Swing Control");

  final_step_ = false;

  // Set Pointer to Control Architecture
  val_ctrl_arch_ = ((ValkyrieControlArchitecture*) _ctrl_arch);
  taf_container_ = val_ctrl_arch_->taf_container_;
  // Get State Provider
  sp_ = ValkyrieStateProvider::getStateProvider(robot_);

  // Set Leg Side
  leg_side_ = _leg_side;
}

SwingControl::~SwingControl(){
}


void SwingControl::firstVisit(){
  std::cout << "Start [Swing Control] Leg Side: " <<  leg_side_ << std::endl;
  // Set control Starting time
  ctrl_start_time_ = sp_->curr_time;

  // Set swing foot trajectory time
  end_time_ = val_ctrl_arch_->dcm_trajectory_manger_->getSwingTime();
  if (leg_side_ == LEFT_ROBOT_SIDE){
    // Set Left Swing Foot Trajectory
  }else{
    // Set Right Foot Swing Trajectory
  }

}

void SwingControl::_taskUpdate(){
  // =========================================================================
  // Set Foot Motion Tasks
  // =========================================================================

  // Update Swing Foot Trajectories.
  val_ctrl_arch_->rfoot_trajectory_manager_->useCurrent();
  val_ctrl_arch_->lfoot_trajectory_manager_->useCurrent();
}

void SwingControl::oneStep(){  
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  _taskUpdate();
}

void SwingControl::lastVisit(){  
  // Increment Step Index
  val_ctrl_arch_->dcm_trajectory_manger_->incrementStepIndex();
}

bool SwingControl::endOfState(){  
  // if time exceeds transition time, switch state
  if (state_machine_time_ >= end_time_){    
    return true;
  }else{
    return false;    
  }
} 

StateIdentifier SwingControl::getNextState(){
  int next_footstep_robot_side;
  // If there's a next footstep, transition to the footstep side.
  if (val_ctrl_arch_->dcm_trajectory_manger_->nextStepRobotSide(next_footstep_robot_side)){
    if (next_footstep_robot_side == LEFT_ROBOT_SIDE){
      return VALKYRIE_STATES::LL_CONTACT_TRANSITION_START;
    }else{
      return VALKYRIE_STATES::RL_CONTACT_TRANSITION_START;
    }
  }else{
    // Otherwise, this must be the last step so transition to double support
    if (leg_side_ == LEFT_ROBOT_SIDE){
      return VALKYRIE_STATES::LL_CONTACT_TRANSITION_START;
    }else {
      return VALKYRIE_STATES::RL_CONTACT_TRANSITION_START;
    }    
  }
}

void SwingControl::initialization(const YAML::Node& node){
}
