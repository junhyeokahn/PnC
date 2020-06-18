#include <PnC/DracoPnC/DracoCtrlArchitecture/DracoCtrlArchitecture.hpp>

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
  joint_trajectory_manager_ =
      new JointTrajectoryManager(taf_container_->joint_task_, robot_);
  floating_base_lifting_up_manager_ = new FloatingBaseTrajectoryManager(
      taf_container_->com_task_, taf_container_->base_ori_task_, robot_);

  rfoot_front_max_normal_force_manager_ = new MaxNormalForceTrajectoryManager(
      taf_container_->rfoot_front_contact_, robot_);
  rfoot_back_max_normal_force_manager_ = new MaxNormalForceTrajectoryManager(
      taf_container_->rfoot_back_contact_, robot_);
  lfoot_front_max_normal_force_manager_ = new MaxNormalForceTrajectoryManager(
      taf_container_->lfoot_front_contact_, robot_);
  lfoot_back_max_normal_force_manager_ = new MaxNormalForceTrajectoryManager(
      taf_container_->lfoot_back_contact_, robot_);
  rfoot_pos_hierarchy_manager_ = new TaskWeightTrajectoryManager(
      taf_container_->rfoot_center_pos_task_, robot_);
  rfoot_ori_hierarchy_manager_ = new TaskWeightTrajectoryManager(
      taf_container_->rfoot_center_ori_task_, robot_);
  lfoot_pos_hierarchy_manager_ = new TaskWeightTrajectoryManager(
      taf_container_->lfoot_center_pos_task_, robot_);
  lfoot_ori_hierarchy_manager_ = new TaskWeightTrajectoryManager(
      taf_container_->lfoot_center_ori_task_, robot_);
  jpos_hierarchy_manager_ =
      new TaskWeightTrajectoryManager(taf_container_->joint_task_, robot_);
  com_hierarchy_manager_ =
      new TaskWeightTrajectoryManager(taf_container_->com_task_, robot_);
  base_ori_hierarchy_manager_ =
      new TaskWeightTrajectoryManager(taf_container_->base_ori_task_, robot_);

  dcm_trajectory_manger_ = new DCMTrajectoryManager(
      dcm_planner_, taf_container_->com_task_, taf_container_->base_ori_task_,
      robot_, DracoBodyNode::lFootCenter, DracoBodyNode::rFootCenter);

  // Initialize states: add all states to the state machine map
  state_machines_[DRACO_STATES::INITIALIZE] =
      new Initialize(DRACO_STATES::INITIALIZE, this, robot_);
  state_machines_[DRACO_STATES::STAND] =
      new DoubleSupportStand(DRACO_STATES::STAND, this, robot_);
  state_machines_[DRACO_STATES::BALANCE] =
      new DoubleSupportBalance(DRACO_STATES::BALANCE, this, robot_);

  state_machines_[DRACO_STATES::RL_CONTACT_TRANSITION_START] =
      new ContactTransitionStart(DRACO_STATES::RL_CONTACT_TRANSITION_START,
                                 RIGHT_ROBOT_SIDE, this, robot_);
  state_machines_[DRACO_STATES::RL_CONTACT_TRANSITION_END] =
      new ContactTransitionEnd(DRACO_STATES::RL_CONTACT_TRANSITION_END,
                               RIGHT_ROBOT_SIDE, this, robot_);
  state_machines_[DRACO_STATES::RL_SWING] =
      new SwingControl(DRACO_STATES::RL_SWING, RIGHT_ROBOT_SIDE, this, robot_);

  state_machines_[DRACO_STATES::LL_CONTACT_TRANSITION_START] =
      new ContactTransitionStart(DRACO_STATES::LL_CONTACT_TRANSITION_START,
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
  delete rfoot_front_max_normal_force_manager_;
  delete rfoot_back_max_normal_force_manager_;
  delete lfoot_front_max_normal_force_manager_;
  delete lfoot_back_max_normal_force_manager_;
  delete dcm_trajectory_manger_;
  delete joint_trajectory_manager_;
  delete floating_base_lifting_up_manager_;

  delete rfoot_pos_hierarchy_manager_;
  delete rfoot_ori_hierarchy_manager_;
  delete lfoot_pos_hierarchy_manager_;
  delete lfoot_ori_hierarchy_manager_;
  delete jpos_hierarchy_manager_;
  delete com_hierarchy_manager_;
  delete base_ori_hierarchy_manager_;

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

  try {
    double max_z_force;
    myUtils::readParameter(cfg_["task_parameters"], "max_z_force", max_z_force);
    rfoot_front_max_normal_force_manager_->setMaxFz(max_z_force);
    rfoot_back_max_normal_force_manager_->setMaxFz(max_z_force);
    lfoot_front_max_normal_force_manager_->setMaxFz(max_z_force);
    lfoot_back_max_normal_force_manager_->setMaxFz(max_z_force);
    double max_gain, min_gain;
    myUtils::readParameter(cfg_["task_parameters"], "max_w_task_com", max_gain);
    myUtils::readParameter(cfg_["task_parameters"], "min_w_task_com", min_gain);
    com_hierarchy_manager_->setMaxGain(max_gain);
    com_hierarchy_manager_->setMinGain(min_gain);
    myUtils::readParameter(cfg_["task_parameters"], "max_w_task_base_ori",
                           max_gain);
    myUtils::readParameter(cfg_["task_parameters"], "min_w_task_base_ori",
                           min_gain);
    base_ori_hierarchy_manager_->setMaxGain(max_gain);
    base_ori_hierarchy_manager_->setMinGain(min_gain);
    myUtils::readParameter(cfg_["task_parameters"], "max_w_task_joint",
                           max_gain);
    myUtils::readParameter(cfg_["task_parameters"], "min_w_task_joint",
                           min_gain);
    jpos_hierarchy_manager_->setMaxGain(max_gain);
    jpos_hierarchy_manager_->setMinGain(min_gain);
    double max_foot_pos_gain, min_foot_pos_gain, max_foot_ori_gain,
        min_foot_ori_gain;
    myUtils::readParameter(cfg_["task_parameters"], "max_w_task_foot_pos",
                           max_foot_pos_gain);
    myUtils::readParameter(cfg_["task_parameters"], "min_w_task_foot_pos",
                           min_foot_pos_gain);
    myUtils::readParameter(cfg_["task_parameters"], "max_w_task_foot_ori",
                           max_foot_ori_gain);
    myUtils::readParameter(cfg_["task_parameters"], "min_w_task_foot_ori",
                           min_foot_ori_gain);
    rfoot_pos_hierarchy_manager_->setMaxGain(max_foot_pos_gain);
    rfoot_pos_hierarchy_manager_->setMinGain(min_foot_pos_gain);
    lfoot_pos_hierarchy_manager_->setMaxGain(max_foot_pos_gain);
    lfoot_pos_hierarchy_manager_->setMinGain(min_foot_pos_gain);
    rfoot_ori_hierarchy_manager_->setMaxGain(max_foot_ori_gain);
    rfoot_ori_hierarchy_manager_->setMinGain(min_foot_ori_gain);
    lfoot_ori_hierarchy_manager_->setMaxGain(max_foot_ori_gain);
    lfoot_ori_hierarchy_manager_->setMinGain(min_foot_ori_gain);

  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }

  dcm_trajectory_manger_->paramInitialization(cfg_["dcm_planner_parameters"]);

  // States Initialization:
  state_machines_[DRACO_STATES::INITIALIZE]->initialization(
      cfg_["state_initialize_params"]);
  state_machines_[DRACO_STATES::STAND]->initialization(
      cfg_["state_stand_params"]);
  state_machines_[DRACO_STATES::RL_SWING]->initialization(cfg_["state_swing"]);
  state_machines_[DRACO_STATES::LL_SWING]->initialization(cfg_["state_swing"]);
}
