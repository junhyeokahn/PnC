#include <PnC/ValkyriePnC/StateMachines/ContactTransition.hpp>
#include <PnC/ValkyriePnC/CtrlArchitectures/ValkyrieControlArchitecture.hpp>

ContactTransition::ContactTransition(const StateIdentifier state_identifier_in, 
                                     const int _leg_side, 
                                     ValkyrieControlArchitecture* _ctrl_arch, RobotSystem* _robot) : 
               StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Contact Transition");

  // Set Pointer to Control Architecture
  val_ctrl_arch_ = ((ValkyrieControlArchitecture*) _ctrl_arch);
  taf_container_ = val_ctrl_arch_->taf_container_;
  // Get State Provider
  sp_ = ValkyrieStateProvider::getStateProvider(robot_);

  // Set Leg Side
  leg_side_ = _leg_side;
}

ContactTransition::~ContactTransition(){
}


void ContactTransition::firstVisit(){
  std::cout << "Contact Transition First Visit" << std::endl;

  ctrl_start_time_ = sp_->curr_time;
  double t_walk_start = ctrl_start_time_;

  // For all contact transitions, initially ramp up the reaction forces to max

  // Check if it's the last footstep

  // If not recompute DCM trajectory:
  //  update transfer time by checking what the previous state is.
  //  check the swing foot type. ramp down the reaction force of the swing foot.


  int transfer_type = DCM_TRANSFER_TYPES::INITIAL;
  // Check if Previous State is From Swing
  if ((val_ctrl_arch_->getPrevState() == VALKYRIE_STATES::RL_SWING) ||
      (val_ctrl_arch_->getPrevState() == VALKYRIE_STATES::LL_SWING) ){
      // Increment Step Index
      val_ctrl_arch_->dcm_trajectory_manger_->incrementStepIndex();   
      // Check if there are no more steps remaining.
      //  if there are no more steps remaining,
      //    set end_time_ to settling time.
      //    set final transition flag.
      //  else
      //    normal contact transition

  }

  // If previous state is from balancing or ending of contact transition, recompute DCM trajectory
  else if ((val_ctrl_arch_->getPrevState() == VALKYRIE_STATES::BALANCE) ||
           (val_ctrl_arch_->getPrevState() == VALKYRIE_STATES::RL_CONTACT_TRANSITION) ||
           (val_ctrl_arch_->getPrevState() == VALKYRIE_STATES::LL_CONTACT_TRANSITION)) {

    // Use Initial transfer time if coming from a balancing state
    if (val_ctrl_arch_->getPrevState() == VALKYRIE_STATES::BALANCE){
        transfer_type = DCM_TRANSFER_TYPES::INITIAL;      
    }else{
    // Otherwise, this is a midstep.
        transfer_type = DCM_TRANSFER_TYPES::MIDSTEP;      
    }

    // 

    // else if previous state is from balance or ending transition
    //       if ending transition is from the opposite foot, use initial transfer time
    //       if ending transition is from the same foot, use contact transition time
    //       initialize the dcm planner
    //       set valid flag.

    // Recompute DCM Trajectories
    Eigen::Quaterniond pelvis_ori(robot_->getBodyNodeCoMIsometry(ValkyrieBodyNode::pelvis).linear());
    val_ctrl_arch_->dcm_trajectory_manger_->initialize(t_walk_start, val_ctrl_arch_->footstep_list_,
                                                       DCM_TRANSFER_TYPES::INITIAL,
                                                       pelvis_ori,
                                                       sp_->dcm, 
                                                       sp_->dcm_vel);

  }






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
