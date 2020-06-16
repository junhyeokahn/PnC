#include <PnC/DracoPnC/DracoCtrlArchitecture/DracoControlArchitecture.hpp>
#include <PnC/DracoPnC/DracoStateMachine/ContactTransitionStart.hpp>

ContactTransitionStart::ContactTransitionStart(
    const StateIdentifier state_identifier_in, const int _leg_side,
    DracoControlArchitecture* _ctrl_arch, RobotSystem* _robot)
    : StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Contact Transition");

  // Set Pointer to Control Architecture
  ctrl_arch_ = ((DracoControlArchitecture*)_ctrl_arch);
  taf_container_ = ctrl_arch_->taf_container_;
  // Get State Provider
  sp_ = DracoStateProvider::getStateProvider(robot_);

  // Set Leg Side
  leg_side_ = _leg_side;
}

ContactTransitionStart::~ContactTransitionStart() {}

void ContactTransitionStart::firstVisit() {
  // Set control Starting time
  ctrl_start_time_ = sp_->curr_time;

  // For all contact transitions, initially ramp up the reaction forces to max
  ctrl_arch_->lfoot_front_max_normal_force_manager_->initializeRampToMax(
      0.0, ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampUpTime());
  ctrl_arch_->lfoot_back_max_normal_force_manager_->initializeRampToMax(
      0.0, ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampUpTime());
  ctrl_arch_->rfoot_front_max_normal_force_manager_->initializeRampToMax(
      0.0, ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampUpTime());
  ctrl_arch_->rfoot_back_max_normal_force_manager_->initializeRampToMax(
      0.0, ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampUpTime());

  // Ramp to max the contact hierarchy weight
  ctrl_arch_->lfoot_pos_hierarchy_manager_->initializeRampToMax(
      0.0, ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampUpTime());
  ctrl_arch_->lfoot_ori_hierarchy_manager_->initializeRampToMax(
      0.0, ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampUpTime());
  ctrl_arch_->rfoot_pos_hierarchy_manager_->initializeRampToMax(
      0.0, ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampUpTime());
  ctrl_arch_->rfoot_ori_hierarchy_manager_->initializeRampToMax(
      0.0, ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampUpTime());

  // Check if it's the last footstep
  if (ctrl_arch_->dcm_trajectory_manger_->noRemainingSteps()) {
    // std::cout << "Final Step. Settling..." << std::endl;
    // If this is the last footstep, then we will just wait until we settle.
    end_time_ =
        ctrl_arch_->dcm_trajectory_manger_->getFinalContactTransferTime();
  } else {
    // std::cout << "Not the last step. Compute DCM trajectory" << std::endl;
    // This is not the last footstep. We need to recompute the remaining DCM
    // trajectories.
    // Set transfer type to midstep
    int transfer_type = DCM_TRANSFER_TYPES::MIDSTEP;
    end_time_ =
        ctrl_arch_->dcm_trajectory_manger_->getMidStepContactTransferTime() -
        ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampDownTime();

    // If coming from a balancing state, use initial transfer time
    if (ctrl_arch_->getPrevState() == DRACO_STATES::BALANCE) {
      transfer_type = DCM_TRANSFER_TYPES::INITIAL;
      end_time_ =
          ctrl_arch_->dcm_trajectory_manger_->getInitialContactTransferTime() -
          ctrl_arch_->dcm_trajectory_manger_->getNormalForceRampDownTime();
    }

    // Recompute DCM Trajectories
    double t_walk_start = ctrl_start_time_;
    Eigen::Quaterniond base_ori(
        robot_->getBodyNodeCoMIsometry(DracoBodyNode::torso).linear());
    ctrl_arch_->dcm_trajectory_manger_->initialize(
        t_walk_start, transfer_type, base_ori, sp_->dcm, sp_->dcm_vel);
  }
}

void ContactTransitionStart::_taskUpdate() {
  // =========================================================================
  // Compute and update new maximum reaction forces
  // =========================================================================
  ctrl_arch_->lfoot_max_normal_force_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->rfoot_max_normal_force_manager_->updateRampToMaxDesired(
      state_machine_time_);

  // =========================================================================
  // Ramp task hierarchy weights
  // =========================================================================
  ctrl_arch_->lfoot_contact_pos_hierarchy_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->lfoot_contact_ori_hierarchy_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->rfoot_contact_pos_hierarchy_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->rfoot_contact_ori_hierarchy_manager_->updateRampToMaxDesired(
      state_machine_time_);

  // =========================================================================
  // Set DCM tasks from trajectory manager
  // =========================================================================
  ctrl_arch_->dcm_trajectory_manger_->updateDCMTasksDesired(sp_->curr_time);

  // =========================================================================
  // Set Foot Motion Tasks
  // =========================================================================
  ctrl_arch_->rfoot_trajectory_manager_->useCurrent();
  ctrl_arch_->lfoot_trajectory_manager_->useCurrent();
}

void ContactTransitionStart::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  _taskUpdate();
}

void ContactTransitionStart::lastVisit() {}

bool ContactTransitionStart::endOfState() {
  // if time exceeds transition time, switch state
  if (state_machine_time_ >= end_time_) {
    return true;
  } else {
    return false;
  }
}

StateIdentifier ContactTransitionStart::getNextState() {
  if (ctrl_arch_->dcm_trajectory_manger_
          ->noRemainingSteps()) {  // or pause walking
    // Return to balancing
    return DRACO_STATES::BALANCE;
  } else {
    if (leg_side_ == LEFT_ROBOT_SIDE) {
      // To left leg contact transition end
      return DRACO_STATES::LL_CONTACT_TRANSITION_END;
    } else if (leg_side_ == RIGHT_ROBOT_SIDE) {
      // To right leg contact transition end
      return DRACO_STATES::RL_CONTACT_TRANSITION_END;
    }
  }
}

void ContactTransitionStart::initialization(const YAML::Node& node) {}
