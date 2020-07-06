#include <PnC/DracoPnC/DracoCtrlArchitecture/DracoCtrlArchitecture.hpp>
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
  if (state_identity_ == DRACO_STATES::RL_CONTACT_TRANSITION_START) {
    std::cout << "[Right Foot Contact Transition Start]" << std::endl;
  } else {
    std::cout << "[Left Foot Contact Transition Start]" << std::endl;
  }
  ctrl_start_time_ = sp_->curr_time;

  // For all contact transitions, initially ramp up the reaction forces to max
  ctrl_arch_->lfoot_front_max_normal_force_manager_->initializeRampToMax(
      0.0, ctrl_arch_->dcm_trajectory_manager_->getNormalForceRampUpTime());
  ctrl_arch_->lfoot_back_max_normal_force_manager_->initializeRampToMax(
      0.0, ctrl_arch_->dcm_trajectory_manager_->getNormalForceRampUpTime());
  ctrl_arch_->rfoot_front_max_normal_force_manager_->initializeRampToMax(
      0.0, ctrl_arch_->dcm_trajectory_manager_->getNormalForceRampUpTime());
  ctrl_arch_->rfoot_back_max_normal_force_manager_->initializeRampToMax(
      0.0, ctrl_arch_->dcm_trajectory_manager_->getNormalForceRampUpTime());

  // Ramp to max the contact hierarchy weight
  ctrl_arch_->lfoot_pos_hierarchy_manager_->initializeRampToMax(
      0.0, ctrl_arch_->dcm_trajectory_manager_->getNormalForceRampUpTime());
  ctrl_arch_->lfoot_ori_hierarchy_manager_->initializeRampToMax(
      0.0, ctrl_arch_->dcm_trajectory_manager_->getNormalForceRampUpTime());
  ctrl_arch_->rfoot_pos_hierarchy_manager_->initializeRampToMax(
      0.0, ctrl_arch_->dcm_trajectory_manager_->getNormalForceRampUpTime());
  ctrl_arch_->rfoot_ori_hierarchy_manager_->initializeRampToMax(
      0.0, ctrl_arch_->dcm_trajectory_manager_->getNormalForceRampUpTime());

  // Check if it's the last footstep
  if (ctrl_arch_->dcm_trajectory_manager_->noRemainingSteps()) {
    // If this is the last footstep, then we will just wait until we settle.
    end_time_ =
        ctrl_arch_->dcm_trajectory_manager_->getFinalContactTransferTime();
  } else {
    // This is not the last footstep. We need to recompute the remaining DCM
    // trajectories.
    // Set transfer type to midstep
    int transfer_type = DCM_TRANSFER_TYPES::MIDSTEP;
    end_time_ =
        ctrl_arch_->dcm_trajectory_manager_->getMidStepContactTransferTime() -
        ctrl_arch_->dcm_trajectory_manager_->getNormalForceRampDownTime();

    // If coming from a balancing state, use initial transfer time
    if (ctrl_arch_->getPrevState() == DRACO_STATES::BALANCE) {
      transfer_type = DCM_TRANSFER_TYPES::INITIAL;
      end_time_ =
          ctrl_arch_->dcm_trajectory_manager_->getInitialContactTransferTime() -
          ctrl_arch_->dcm_trajectory_manager_->getNormalForceRampDownTime();
    }

    // Recompute DCM Trajectories
    double t_walk_start = ctrl_start_time_;
    Eigen::Quaterniond base_ori(
        robot_->getBodyNodeCoMIsometry(DracoBodyNode::Torso).linear());

    if (ctrl_arch_->getPrevState() == DRACO_STATES::BALANCE) {
      ctrl_arch_->dcm_trajectory_manager_->initialize(
          t_walk_start, transfer_type, base_ori, sp_->dcm, sp_->dcm_vel);
    }

    // std::cout << "com pos" << std::endl;
    // std::cout << sp_->com_pos << std::endl;
    // std::cout << "com vel" << std::endl;
    // std::cout << sp_->com_vel << std::endl;
    // std::cout << "omega" << std::endl;
    // std::cout << sp_->dcm_omega << std::endl;

    ctrl_arch_->dcm_trajectory_manager_->saveSolution(
        std::to_string(sp_->planning_id));
    sp_->planning_id += 1;
  }
}

void ContactTransitionStart::_taskUpdate() {
  // =========================================================================
  // Floating Base
  // =========================================================================
  ctrl_arch_->dcm_trajectory_manager_->updateDCMTasksDesired(sp_->curr_time);

  // =========================================================================
  // Foot
  // =========================================================================
  ctrl_arch_->rfoot_trajectory_manager_->useCurrent();
  ctrl_arch_->lfoot_trajectory_manager_->useCurrent();
}

void ContactTransitionStart::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // =========================================================================
  // Compute and update new maximum reaction forces
  // =========================================================================
  ctrl_arch_->lfoot_front_max_normal_force_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->lfoot_back_max_normal_force_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->rfoot_front_max_normal_force_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->rfoot_back_max_normal_force_manager_->updateRampToMaxDesired(
      state_machine_time_);

  // =========================================================================
  // Ramp task hierarchy weights
  // =========================================================================
  ctrl_arch_->lfoot_pos_hierarchy_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->lfoot_ori_hierarchy_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->rfoot_pos_hierarchy_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->rfoot_ori_hierarchy_manager_->updateRampToMaxDesired(
      state_machine_time_);

  _taskUpdate();
}

void ContactTransitionStart::lastVisit() {}

bool ContactTransitionStart::endOfState() {
  if (state_machine_time_ >= end_time_) {
    return true;
  } else {
    return false;
  }
}

StateIdentifier ContactTransitionStart::getNextState() {
  if (ctrl_arch_->dcm_trajectory_manager_->noRemainingSteps()) {
    return DRACO_STATES::BALANCE;
  } else {
    if (leg_side_ == LEFT_ROBOT_SIDE) {
      sp_->stance_foot = DracoBodyNode::rFootCenter;
      return DRACO_STATES::LL_CONTACT_TRANSITION_END;
    } else if (leg_side_ == RIGHT_ROBOT_SIDE) {
      sp_->stance_foot = DracoBodyNode::lFootCenter;
      return DRACO_STATES::RL_CONTACT_TRANSITION_END;
    }
  }
}

void ContactTransitionStart::initialization(const YAML::Node& node) {}
