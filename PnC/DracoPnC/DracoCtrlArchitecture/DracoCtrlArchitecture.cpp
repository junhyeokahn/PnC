#include <PnC/DracoPnC/DracoCtrlArchitecture/DracoControlArchitecture.hpp>

DracoControlArchitecture::DracoControlArchitecture(RobotSystem* _robot)
    : ControlArchitecture(_robot) {
  b_state_first_visit_ = true;

  myUtils::pretty_constructor(1, "Draco Control Architecture");
  cfg_ = YAML::LoadFile(
      THIS_COM "Config/Draco/TEST/WALKING_CONTROL_ARCHITECTURE_PARAMS.yaml");

  sp_ = DracoStateProvider::getStateProvider(robot_);

  // Initialize Main Controller
  taf_container_ = new DracoTaskAndForceContainer(robot_);
  main_controller_ = new DracoMainController(taf_container_, robot_);
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

  rfoot_contact_pos_hierarchy_manager_ = new TaskGainScheduleTrajectoryManager(
      taf_container_->rfoot_center_pos_task_, robot_);
  rfoot_contact_ori_hierarchy_manager_ = new TaskGainScheduleTrajectoryManager(
      taf_container_->rfoot_center_ori_task_, robot_);
  lfoot_contact_pos_hierarchy_manager_ = new TaskGainScheduleTrajectoryManager(
      taf_container_->lfoot_center_pos_task_, robot_);
  lfoot_contact_ori_hierarchy_manager_ = new TaskGainScheduleTrajectoryManager(
      taf_container_->lfoot_center_ori_task_, robot_);

  dcm_trajectory_manger_ = new DCMPlannerTrajectoryManager(
      dcm_planner_, taf_container_->com_task_, taf_container_->torso_ori_task_,
      robot_, DracoBodyNode::lFootCenter, DracoBodyNode::rFootCenter);

  // Initialize states: add all states to the state machine map
  state_machines_[DRACO_STATES::INITIALIZE] =
      new Initialize(DRACO_STATES::INITIALIZE, this, robot_);
  state_machines_[DRACO_STATES::STAND] =
      new DoubleSupportStand(DRACO_STATES::STAND, this, robot_);
  state_machines_[DRACO_STATES::BALANCE] =
      new DoubleSupportBalance(DRACO_STATES::BALANCE, this, robot_);

  state_machines_[DRACO_STATES::RL_CONTACT_TRANSITION_START] =
      new ContactTransition(DRACO_STATES::RL_CONTACT_TRANSITION_START,
                            RIGHT_ROBOT_SIDE, this, robot_);
  state_machines_[DRACO_STATES::RL_CONTACT_TRANSITION_END] =
      new ContactTransitionEnd(DRACO_STATES::RL_CONTACT_TRANSITION_END,
                               RIGHT_ROBOT_SIDE, this, robot_);
  state_machines_[DRACO_STATES::RL_SWING] =
      new SwingControl(DRACO_STATES::RL_SWING, RIGHT_ROBOT_SIDE, this, robot_);

  state_machines_[DRACO_STATES::LL_CONTACT_TRANSITION_START] =
      new ContactTransition(DRACO_STATES::LL_CONTACT_TRANSITION_START,
                            LEFT_ROBOT_SIDE, this, robot_);
  state_machines_[DRACO_STATES::LL_CONTACT_TRANSITION_END] =
      new ContactTransitionEnd(DRACO_STATES::LL_CONTACT_TRANSITION_END,
                               LEFT_ROBOT_SIDE, this, robot_);
  state_machines_[DRACO_STATES::LL_SWING] =
      new SwingControl(DRACO_STATES::LL_SWING, LEFT_ROBOT_SIDE, this, robot_);

  // Set Starting State
  state_ = DRACO_STATES::INITIALIZE;
  prev_state_ = state_;

  _InitializeParameters();
}

DracoControlArchitecture::~DracoControlArchitecture() {
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
  delete state_machines_[DRACO_STATES::INITIALIZE];
  delete state_machines_[DRACO_STATES::STAND];
  delete state_machines_[DRACO_STATES::BALANCE];
  delete state_machines_[DRACO_STATES::RL_CONTACT_TRANSITION_START];
  delete state_machines_[DRACO_STATES::RL_CONTACT_TRANSITION_END];
  delete state_machines_[DRACO_STATES::RL_SWING];
  delete state_machines_[DRACO_STATES::LL_CONTACT_TRANSITION_START];
  delete state_machines_[DRACO_STATES::LL_CONTACT_TRANSITION_END];
  delete state_machines_[DRACO_STATES::LL_SWING];
}

void DracoControlArchitecture::ControlArchitectureInitialization() {}

void DracoControlArchitecture::getCommand(void* _command) {
  // Initialize State
  if (b_state_first_visit_) {
    state_machines_[state_]->firstVisit();
    b_state_first_visit_ = false;
  }

  // Update State Machine
  state_machines_[state_]->oneStep();
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

void DracoControlArchitecture::_InitializeParameters() {
  taf_container_->paramInitialization(cfg_["task_parameters"]);
  main_controller_->ctrlInitialization(cfg_["controller_parameters"]);

  // Trajectory Managers initialization
  rfoot_trajectory_manager_->paramInitialization(
      cfg_["foot_trajectory_parameters"]);
  lfoot_trajectory_manager_->paramInitialization(
      cfg_["foot_trajectory_parameters"]);
  lfoot_max_normal_force_manager_->paramInitialization(cfg_["task_parameters"]);
  rfoot_max_normal_force_manager_->paramInitialization(cfg_["task_parameters"]);

  rfoot_contact_pos_hierarchy_manager_->paramInitialization(
      cfg_["task_parameters"]);
  rfoot_contact_ori_hierarchy_manager_->paramInitialization(
      cfg_["task_parameters"]);
  lfoot_contact_pos_hierarchy_manager_->paramInitialization(
      cfg_["task_parameters"]);
  lfoot_contact_ori_hierarchy_manager_->paramInitialization(
      cfg_["task_parameters"]);

  dcm_trajectory_manger_->paramInitialization(cfg_["dcm_planner_parameters"]);

  // States Initialization:
  state_machines_[DRACO_STATES::STAND]->initialization(
      cfg_["state_stand_params"]);
  state_machines_[DRACO_STATES::RL_SWING]->initialization(cfg_["state_swing"]);
  state_machines_[DRACO_STATES::LL_SWING]->initialization(cfg_["state_swing"]);
}
