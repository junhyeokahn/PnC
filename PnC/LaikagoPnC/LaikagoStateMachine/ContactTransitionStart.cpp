#include <PnC/LaikagoPnC/LaikagoCtrlArchitecture/LaikagoCtrlArchitecture.hpp>
#include <PnC/LaikagoPnC/LaikagoStateMachine/ContactTransitionStart.hpp>

ContactTransitionStart::ContactTransitionStart(
    const StateIdentifier state_identifier_in, const int _leg_side,
    LaikagoControlArchitecture* _ctrl_arch, RobotSystem* _robot)
    : StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Contact Transition");

  // Set Pointer to Control Architecture
  ctrl_arch_ = ((LaikagoControlArchitecture*)_ctrl_arch);
  taf_container_ = ctrl_arch_->taf_container_;
  // Get State Provider
  sp_ = LaikagoStateProvider::getStateProvider(robot_);

  // Set Leg Side
  leg_side_ = _leg_side;
}

ContactTransitionStart::~ContactTransitionStart() {}

void ContactTransitionStart::firstVisit() {
  // Set control Starting time
  if (state_identity_ == Laikago_STATES::RL_CONTACT_TRANSITION_START) {
    std::cout << "[Right Foot Contact Transition Start]" << std::endl;
  } else {
    std::cout << "[Left Foot Contact Transition Start]" << std::endl;
  }
  ctrl_start_time_ = sp_->curr_time;

  // For all contact transitions, initially ramp up the reaction forces to max
  /*  ctrl_arch_->lfoot_front_max_normal_force_manager_->initializeRampToMax(*/
  // 0.0, ctrl_arch_->dcm_trajectory_manager_->getNormalForceRampUpTime());
  // ctrl_arch_->lfoot_back_max_normal_force_manager_->initializeRampToMax(
  // 0.0, ctrl_arch_->dcm_trajectory_manager_->getNormalForceRampUpTime());
  // ctrl_arch_->rfoot_front_max_normal_force_manager_->initializeRampToMax(
  // 0.0, ctrl_arch_->dcm_trajectory_manager_->getNormalForceRampUpTime());
  // ctrl_arch_->rfoot_back_max_normal_force_manager_->initializeRampToMax(
  /*0.0, ctrl_arch_->dcm_trajectory_manager_->getNormalForceRampUpTime());*/
  ctrl_arch_->rfoot_max_normal_force_manager_->initializeRampToMax(
      0.0, ctrl_arch_->dcm_trajectory_manager_->getNormalForceRampUpTime());
  ctrl_arch_->lfoot_max_normal_force_manager_->initializeRampToMax(
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
    if (ctrl_arch_->getPrevState() == Laikago_STATES::BALANCE) {
      transfer_type = DCM_TRANSFER_TYPES::INITIAL;
      end_time_ =
          ctrl_arch_->dcm_trajectory_manager_->getInitialContactTransferTime() -
          ctrl_arch_->dcm_trajectory_manager_->getNormalForceRampDownTime();
    }

    // Recompute DCM Trajectories
    double t_walk_start = ctrl_start_time_;
    Eigen::Quaterniond base_ori(
        robot_->getBodyNodeCoMIsometry(LaikagoBodyNode::trunk).linear());

    if (ctrl_arch_->getPrevState() == Laikago_STATES::BALANCE) {
      ctrl_arch_->dcm_trajectory_manager_->initialize(
          t_walk_start, transfer_type, base_ori, sp_->dcm, sp_->dcm_vel);
    }

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
  /*  ctrl_arch_->lfoot_front_max_normal_force_manager_->updateRampToMaxDesired(*/
  // state_machine_time_);
  // ctrl_arch_->lfoot_back_max_normal_force_manager_->updateRampToMaxDesired(
  // state_machine_time_);
  // ctrl_arch_->rfoot_front_max_normal_force_manager_->updateRampToMaxDesired(
  // state_machine_time_);
  // ctrl_arch_->rfoot_back_max_normal_force_manager_->updateRampToMaxDesired(
  /*state_machine_time_);*/
  ctrl_arch_->rfoot_max_normal_force_manager_->updateRampToMaxDesired(
      state_machine_time_);
  ctrl_arch_->lfoot_max_normal_force_manager_->updateRampToMaxDesired(
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
    return Laikago_STATES::BALANCE;
  } else {
    if (leg_side_ == LEFT_ROBOT_SIDE) {
      sp_->stance_foot = LaikagoBodyNode::FR_foot;
      return Laikago_STATES::LL_CONTACT_TRANSITION_END;
    } else if (leg_side_ == RIGHT_ROBOT_SIDE) {
      sp_->stance_foot = LaikagoBodyNode::FL_foot;
      return Laikago_STATES::RL_CONTACT_TRANSITION_END;
    }
  }
}

void ContactTransitionStart::initialization(const YAML::Node& node) {}
