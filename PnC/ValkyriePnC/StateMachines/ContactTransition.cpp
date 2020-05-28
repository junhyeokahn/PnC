#include <PnC/ValkyriePnC/StateMachines/ContactTransition.hpp>
#include <PnC/ValkyriePnC/CtrlArchitectures/ValkyrieControlArchitecture.hpp>

ContactTransition::ContactTransition(const StateIdentifier state_identifier_in, ValkyrieControlArchitecture* _ctrl_arch, RobotSystem* _robot) : 
               StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Contact Transition");

  // Set Pointer to Control Architecture
  val_ctrl_arch_ = ((ValkyrieControlArchitecture*) _ctrl_arch);
  taf_container_ = val_ctrl_arch_->taf_container_;
  // Get State Provider
  sp_ = ValkyrieStateProvider::getStateProvider(robot_);

}

ContactTransition::~ContactTransition(){
}


void ContactTransition::firstVisit(){
  std::cout << "Contact Transition First Visit" << std::endl;

  ctrl_start_time_ = sp_->curr_time;
  double t_walk_start = ctrl_start_time_;

  // if previous state is from swing,
  //  incrementStepIndex()

  //  if there are no more steps remaining,
  //    set end_time_ to settling time.
  //    set final transition flag.
  //  else
  //    normal contact transition

  // else if previous state is from balance or ending transition
  //       if ending transition is from the opposite foot, use initial transfer time
  //       if ending transition is from the same foot, use contact transition time
  //       initialize the dcm planner
  //       set valid flag.

  // Initialize DCM planner
  double t_transfer = 0.1;
  Eigen::Quaterniond pelvis_ori(robot_->getBodyNodeCoMIsometry(ValkyrieBodyNode::pelvis).linear());
  val_ctrl_arch_->dcm_trajectory_manger_->initialize(t_walk_start, val_ctrl_arch_->footstep_list_,
                                                     t_transfer,
                                                     pelvis_ori,
                                                     sp_->dcm, 
                                                     sp_->dcm_vel);

}

void ContactTransition::_taskUpdate(){
  // Get DCM tasks from trajectory manager


  // =========================================================================
  // Set Foot Motion Tasks
  // =========================================================================
  val_ctrl_arch_->rfoot_trajectory_manager_->useCurrent();
  val_ctrl_arch_->lfoot_trajectory_manager_->useCurrent();
}

void ContactTransition::oneStep(){  
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  _taskUpdate();
}

void ContactTransition::lastVisit(){  
}

bool ContactTransition::endOfState(){  
  // if time exceeds transition time, switch state
  return false;
} 

StateIdentifier ContactTransition::getNextState(){
  // if no more steps remaining, transition to balance. 
  // otherwise, check next step.
  // if next step is for the opposite foot, transition to left leg start
  // if next step is for the same foot, transition to right leg start

  // if pause trajectory is hit, transition to balance 

}


void ContactTransition::initialization(const YAML::Node& node){
}
