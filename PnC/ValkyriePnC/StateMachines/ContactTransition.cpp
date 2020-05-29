#include <PnC/ValkyriePnC/StateMachines/ContactTransition.hpp>
#include <PnC/ValkyriePnC/CtrlArchitectures/ValkyrieControlArchitecture.hpp>

ContactTransition::ContactTransition(const StateIdentifier state_identifier_in, 
                                     const int _leg_side, 
                                     ValkyrieControlArchitecture* _ctrl_arch, RobotSystem* _robot) : 
               StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Contact Transition");

  final_step_ = false;

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
  // Set control Starting time
  ctrl_start_time_ = sp_->curr_time;

  // For all contact transitions, initially ramp up the reaction forces to max
  val_ctrl_arch_->lfoot_max_normal_force_manager_->initializeRampToMax(0.0, val_ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampUpTime());
  val_ctrl_arch_->rfoot_max_normal_force_manager_->initializeRampToMax(0.0, val_ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampUpTime());

  // Check if it's the last footstep
  if (val_ctrl_arch_->dcm_trajectory_manger_->noRemainingSteps()){
    // If this is the last footstep, then we will just wait until we settle.
    end_time_ = val_ctrl_arch_->dcm_trajectory_manger_->getFinalContactTransferTime();
    final_step_ = true;  // set flag.  
  }else{
    // This is not the last footstep. We need to recompute the remaining DCM trajectories.
    // Set transfer type to midstep
    int transfer_type = DCM_TRANSFER_TYPES::MIDSTEP;
    end_time_ = val_ctrl_arch_->dcm_trajectory_manger_->getMidStepContactTransferTime() -
                val_ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampDownTime();

    // If coming from a balancing state, use initial transfer time 
    if (val_ctrl_arch_->getPrevState() == VALKYRIE_STATES::BALANCE){
        transfer_type = DCM_TRANSFER_TYPES::INITIAL;      
        end_time_ = val_ctrl_arch_->dcm_trajectory_manger_->getInitialContactTransferTime() -
                    val_ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampDownTime();
    }

    // Recompute DCM Trajectories
    double t_walk_start = ctrl_start_time_;
    Eigen::Quaterniond pelvis_ori(robot_->getBodyNodeCoMIsometry(ValkyrieBodyNode::pelvis).linear());
    val_ctrl_arch_->dcm_trajectory_manger_->initialize(t_walk_start,
                                                       transfer_type,
                                                       pelvis_ori,
                                                       sp_->dcm, 
                                                       sp_->dcm_vel);
  }

}

void ContactTransition::_taskUpdate(){
  // =========================================================================
  // Compute and update new maximum reaction forces
  // =========================================================================
  val_ctrl_arch_->lfoot_max_normal_force_manager_->updateRampToMaxDesired(state_machine_time_);
  val_ctrl_arch_->rfoot_max_normal_force_manager_->updateRampToMaxDesired(state_machine_time_);

  // =========================================================================
  // Get DCM tasks from trajectory manager 
  // =========================================================================


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
  // reset flags
  final_step_ = false;
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

  // if leg_side_ is RIGHT -> transition to right foot end contact transition
  // if leg_side_ is LEFT -> transition to left foot enc contact transition

  // if last footstep or pause trajectory is activated -> transition to 
  // if pause trajectory is hit, transition to balance 

}


void ContactTransition::initialization(const YAML::Node& node){
}
