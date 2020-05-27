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
  double t_walk_start = ctrl_start_time_;

  // Initialize DCM planner
  val_ctrl_arch_->dcm_planner_->setInitialTime(t_walk_start);
  // val_ctrl_arch_->dcm_planner_->setInitialOri(x_ori_start_);
  // val_ctrl_arch_->dcm_planner_->initialize_footsteps_rvrp(footstep_list_, left_foot_start_,
                                                          // right_foot_start_, dcm_pos_start_in, dcm_vel_start_in);
}

void InitialTransfer::_taskUpdate(){
  // =========================================================================
  // Set Foot Motion Tasks
  // =========================================================================
  val_ctrl_arch_->rfoot_trajectory_manager_->useCurrent();
  val_ctrl_arch_->lfoot_trajectory_manager_->useCurrent();
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
