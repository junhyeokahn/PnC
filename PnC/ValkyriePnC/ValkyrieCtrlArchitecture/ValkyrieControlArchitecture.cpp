#include <PnC/ValkyriePnC/ValkyrieCtrlArchitecture/ValkyrieControlArchitecture.hpp>

ValkyrieControlArchitecture::ValkyrieControlArchitecture(RobotSystem* _robot)
    : ControlArchitecture(_robot) {
  b_state_first_visit_ = true;

  myUtils::pretty_constructor(1, "Valkyrie Control Architecture");
  cfg_ = YAML::LoadFile(
      THIS_COM "Config/Valkyrie/TEST/CONTROL_ARCHITECTURE_PARAMS.yaml");

  sp_ = ValkyrieStateProvider::getStateProvider(robot_);

  // Initialize Main Controller
  taf_container_ = new ValkyrieTaskAndForceContainer(robot_);
  main_controller_ = new ValkyrieMainController(taf_container_, robot_);
  // Initialize Planner
  dcm_planner_ = new DCMPlanner();

  // Initialize Trajectory managers
  rfoot_trajectory_manager_ = new FootSE3TrajectoryManager(
      taf_container_->rfoot_center_pos_task_,
      taf_container_->rfoot_center_ori_task_, robot_);
  lfoot_trajectory_manager_ = new FootSE3TrajectoryManager(
      taf_container_->lfoot_center_pos_task_,
      taf_container_->lfoot_center_ori_task_, robot_);
  rfoot_max_normal_force_manager_ = new MaxNormalForceTrajectoryManager(
      taf_container_->rfoot_contact_, robot_);
  lfoot_max_normal_force_manager_ = new MaxNormalForceTrajectoryManager(
      taf_container_->lfoot_contact_, robot_);
  upper_body_joint_trajectory_manager_ = new UpperBodyJointTrajectoryManager(
      taf_container_->upper_body_task_, robot_);

  rfoot_contact_pos_hierarchy_manager_ = new TaskWeightTrajectoryManager(
      taf_container_->rfoot_center_pos_task_, robot_);
  rfoot_contact_ori_hierarchy_manager_ = new TaskWeightTrajectoryManager(
      taf_container_->rfoot_center_ori_task_, robot_);
  lfoot_contact_pos_hierarchy_manager_ = new TaskWeightTrajectoryManager(
      taf_container_->lfoot_center_pos_task_, robot_);
  lfoot_contact_ori_hierarchy_manager_ = new TaskWeightTrajectoryManager(
      taf_container_->lfoot_center_ori_task_, robot_);

  dcm_trajectory_manger_ = new DCMTrajectoryManager(
      dcm_planner_, taf_container_->com_task_, taf_container_->pelvis_ori_task_,
      robot_, ValkyrieBodyNode::leftCOP_Frame,
      ValkyrieBodyNode::rightCOP_Frame);

  // Initialize states: add all states to the state machine map
  state_machines_[VALKYRIE_STATES::STAND] =
      new DoubleSupportStand(VALKYRIE_STATES::STAND, this, robot_);
  state_machines_[VALKYRIE_STATES::BALANCE] =
      new DoubleSupportBalance(VALKYRIE_STATES::BALANCE, this, robot_);

  state_machines_[VALKYRIE_STATES::RL_CONTACT_TRANSITION_START] =
      new ContactTransition(VALKYRIE_STATES::RL_CONTACT_TRANSITION_START,
                            RIGHT_ROBOT_SIDE, this, robot_);
  state_machines_[VALKYRIE_STATES::RL_CONTACT_TRANSITION_END] =
      new ContactTransitionEnd(VALKYRIE_STATES::RL_CONTACT_TRANSITION_END,
                               RIGHT_ROBOT_SIDE, this, robot_);
  state_machines_[VALKYRIE_STATES::RL_SWING] = new SwingControl(
      VALKYRIE_STATES::RL_SWING, RIGHT_ROBOT_SIDE, this, robot_);

  state_machines_[VALKYRIE_STATES::LL_CONTACT_TRANSITION_START] =
      new ContactTransition(VALKYRIE_STATES::LL_CONTACT_TRANSITION_START,
                            LEFT_ROBOT_SIDE, this, robot_);
  state_machines_[VALKYRIE_STATES::LL_CONTACT_TRANSITION_END] =
      new ContactTransitionEnd(VALKYRIE_STATES::LL_CONTACT_TRANSITION_END,
                               LEFT_ROBOT_SIDE, this, robot_);
  state_machines_[VALKYRIE_STATES::LL_SWING] = new SwingControl(
      VALKYRIE_STATES::LL_SWING, LEFT_ROBOT_SIDE, this, robot_);

  // Set Starting State
  state_ = VALKYRIE_STATES::STAND;
  prev_state_ = state_;

  _InitializeParameters();
}

ValkyrieControlArchitecture::~ValkyrieControlArchitecture() {
  delete taf_container_;
  delete main_controller_;
  delete dcm_planner_;

  // Delete the trajectory managers
  delete rfoot_trajectory_manager_;
  delete lfoot_trajectory_manager_;
  delete rfoot_max_normal_force_manager_;
  delete lfoot_max_normal_force_manager_;
  delete dcm_trajectory_manger_;

  delete rfoot_contact_pos_hierarchy_manager_;
  delete rfoot_contact_ori_hierarchy_manager_;
  delete lfoot_contact_pos_hierarchy_manager_;
  delete lfoot_contact_ori_hierarchy_manager_;

  // Delete the state machines
  delete state_machines_[VALKYRIE_STATES::STAND];
  delete state_machines_[VALKYRIE_STATES::BALANCE];
  delete state_machines_[VALKYRIE_STATES::RL_CONTACT_TRANSITION_START];
  delete state_machines_[VALKYRIE_STATES::RL_CONTACT_TRANSITION_END];
  delete state_machines_[VALKYRIE_STATES::RL_SWING];
  delete state_machines_[VALKYRIE_STATES::LL_CONTACT_TRANSITION_START];
  delete state_machines_[VALKYRIE_STATES::LL_CONTACT_TRANSITION_END];
  delete state_machines_[VALKYRIE_STATES::LL_SWING];
}

void ValkyrieControlArchitecture::ControlArchitectureInitialization() {}

void ValkyrieControlArchitecture::getCommand(void* _command) {
  // Initialize State
  if (b_state_first_visit_) {
    state_machines_[state_]->firstVisit();
    b_state_first_visit_ = false;
  }

  // Update State Machine
  state_machines_[state_]->oneStep();
  // Update Desired of state Independent trajectories
  upper_body_joint_trajectory_manager_->updateDesired();
  // Get Wholebody control commands
  main_controller_->getCommand(_command);

  // Check for State Transitions
  if (state_machines_[state_]->endOfState()) {
    state_machines_[state_]->lastVisit();
    prev_state_ = state_;
    state_ = state_machines_[state_]->getNextState();
    b_state_first_visit_ = true;
  }
};

void ValkyrieControlArchitecture::_InitializeParameters() {
  taf_container_->paramInitialization(cfg_["task_parameters"]);
  main_controller_->ctrlInitialization(cfg_["controller_parameters"]);

  // Trajectory Managers initialization
  rfoot_trajectory_manager_->paramInitialization(
      cfg_["foot_trajectory_parameters"]);
  lfoot_trajectory_manager_->paramInitialization(
      cfg_["foot_trajectory_parameters"]);

  lfoot_max_normal_force_manager_->setMaxFz(1500.);
  rfoot_max_normal_force_manager_->setMaxFz(1500.);
  rfoot_contact_pos_hierarchy_manager_->setMaxGain(40.0);
  rfoot_contact_pos_hierarchy_manager_->setMinGain(20.0);
  rfoot_contact_ori_hierarchy_manager_->setMaxGain(40.0);
  rfoot_contact_ori_hierarchy_manager_->setMinGain(20.0);
  lfoot_contact_pos_hierarchy_manager_->setMaxGain(40.0);
  lfoot_contact_pos_hierarchy_manager_->setMinGain(20.0);
  lfoot_contact_ori_hierarchy_manager_->setMaxGain(40.0);
  lfoot_contact_ori_hierarchy_manager_->setMinGain(20.0);

  dcm_trajectory_manger_->paramInitialization(cfg_["dcm_planner_parameters"]);

  // States Initialization:
  state_machines_[VALKYRIE_STATES::STAND]->initialization(
      cfg_["state_stand_params"]);
  state_machines_[VALKYRIE_STATES::RL_SWING]->initialization(
      cfg_["state_swing"]);
  state_machines_[VALKYRIE_STATES::LL_SWING]->initialization(
      cfg_["state_swing"]);
}
