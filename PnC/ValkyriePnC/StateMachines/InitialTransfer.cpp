#include <PnC/ValkyriePnC/StateMachines/InitialTransfer.hpp>
#include <PnC/ValkyriePnC/CtrlArchitectures/ValkyrieControlArchitecture.hpp>

InitialTransfer::InitialTransfer(const StateIdentifier state_identifier_in, ValkyrieControlArchitecture* _ctrl_arch, RobotSystem* _robot) : 
               StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Initial Transfer");

  // Set Pointer to Control Architecture
  val_ctrl_arch_ = ((ValkyrieControlArchitecture*) _ctrl_arch);
  taf_container_ = val_ctrl_arch_->taf_container_;
  // Get State Provider
  sp_ = ValkyrieStateProvider::getStateProvider(robot_);

}

InitialTransfer::~InitialTransfer(){
}


void InitialTransfer::firstVisit(){
  std::cout << "Initial Transfer First Visit" << std::endl;

  ctrl_start_time_ = sp_->curr_time;
  double t_walk_start = ctrl_start_time_;

  // Initialize DCM planner
  double t_transfer = 0.1;
  Eigen::Quaterniond pelvis_ori(robot_->getBodyNodeCoMIsometry(ValkyrieBodyNode::pelvis).linear());
  val_ctrl_arch_->dcm_trajectory_manger_->initialize(t_walk_start, val_ctrl_arch_->footstep_list_,
                                                             t_transfer,
                                                             pelvis_ori,
                                                             sp_->dcm, 
                                                             sp_->dcm_vel);

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
