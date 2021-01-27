#include <PnC/A1PnC/A1CtrlArchitecture/A1CtrlArchitecture.hpp>

A1ControlArchitecture::A1ControlArchitecture(RobotSystem* _robot)
    : ControlArchitecture(_robot) {
  b_state_first_visit_ = true;

  myUtils::pretty_constructor(1, "A1 Control Architecture");
  cfg_ = YAML::LoadFile(THIS_COM "Config/A1/TEST/CONTROL_ARCHITECTURE_PARAMS.yaml");

  sp_ = A1StateProvider::getStateProvider(robot_);

  // Initialize Main Controller
  taf_container_ = new A1TaskAndForceContainer(robot_);
  taf_container_->paramInitialization(cfg_["task_parameters"]);
  // main_controller_ = new A1MainController(taf_container_, robot_);

  // Initialize Planner

  // Initialize Trajectory managers
  floating_base_lifting_up_manager_ = new FloatingBaseTrajectoryManager(
      taf_container_->com_task_, taf_container_->base_ori_task_, robot_);

  frfoot_max_normal_force_manager_ = new MaxNormalForceTrajectoryManager(
            taf_container_->frfoot_contact_, robot_);
  rrfoot_max_normal_force_manager_ = new MaxNormalForceTrajectoryManager(
            taf_container_->rrfoot_contact_, robot_);
  flfoot_max_normal_force_manager_ = new MaxNormalForceTrajectoryManager(
            taf_container_->flfoot_contact_, robot_);
  rlfoot_max_normal_force_manager_ = new MaxNormalForceTrajectoryManager(
            taf_container_->rlfoot_contact_, robot_);

  frfoot_pos_hierarchy_manager_ = new TaskWeightTrajectoryManager(
      taf_container_->frfoot_pos_task_, robot_);
  rrfoot_pos_hierarchy_manager_ = new TaskWeightTrajectoryManager(
      taf_container_->rrfoot_pos_task_, robot_);
  flfoot_pos_hierarchy_manager_ = new TaskWeightTrajectoryManager(
      taf_container_->flfoot_pos_task_, robot_);
  rlfoot_pos_hierarchy_manager_ = new TaskWeightTrajectoryManager(
      taf_container_->rlfoot_pos_task_, robot_);

  com_hierarchy_manager_ =
      new TaskWeightTrajectoryManager(taf_container_->com_task_, robot_);
  base_ori_hierarchy_manager_ =
      new TaskWeightTrajectoryManager(taf_container_->base_ori_task_, robot_);

  // Initialize states: add all states to the state machine map
  state_machines_[A1_STATES::INITIALIZE] =
      new Initialize(A1_STATES::INITIALIZE, this, robot_);
  /*state_machines_[DRACO_STATES::STAND] =
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
      new SwingControl(DRACO_STATES::LL_SWING, LEFT_ROBOT_SIDE, this, robot_);*/

  // Set Starting State
  state_ = A1_STATES::INITIALIZE;
  prev_state_ = state_;

  _InitializeParameters();
}

A1ControlArchitecture::~A1ControlArchitecture() {
  delete taf_container_;
  // delete main_controller_;

  // Delete the trajectory managers
  delete frfoot_max_normal_force_manager_;
  delete rrfoot_max_normal_force_manager_;
  delete flfoot_max_normal_force_manager_;
  delete rlfoot_max_normal_force_manager_;
  // delete joint_trajectory_manager_;
  delete floating_base_lifting_up_manager_;

  delete frfoot_pos_hierarchy_manager_;
  delete rrfoot_pos_hierarchy_manager_;
  delete rlfoot_pos_hierarchy_manager_;
  delete flfoot_pos_hierarchy_manager_;
  delete com_hierarchy_manager_;
  delete base_ori_hierarchy_manager_;

  // Delete the state machines
  delete state_machines_[A1_STATES::INITIALIZE];
  // delete state_machines_[DRACO_STATES::STAND];
  // delete state_machines_[DRACO_STATES::BALANCE];
  // delete state_machines_[DRACO_STATES::RL_CONTACT_TRANSITION_START];
  // delete state_machines_[DRACO_STATES::RL_CONTACT_TRANSITION_END];
  // delete state_machines_[DRACO_STATES::RL_SWING];
  // delete state_machines_[DRACO_STATES::LL_CONTACT_TRANSITION_START];
  // delete state_machines_[DRACO_STATES::LL_CONTACT_TRANSITION_END];
  // delete state_machines_[DRACO_STATES::LL_SWING];
}

void A1ControlArchitecture::ControlArchitectureInitialization() {}

void A1ControlArchitecture::getCommand(void* _command) {
  // Initialize State
  if (b_state_first_visit_) {
    state_machines_[state_]->firstVisit();
    b_state_first_visit_ = false;
  }

  // Update State Machine
  state_machines_[state_]->oneStep();
  // Get Wholebody control commands
  // main_controller_->getCommand(_command);
  /*if (state_ == A1_STATES::INITIALIZE) {
    getIVDCommand(_command);
  } else {
    if (state_ == DRACO_STATES::BALANCE &&
        (floating_base_lifting_up_manager_->is_swaying ||
         floating_base_lifting_up_manager_->is_sinusoid)) {
      floating_base_lifting_up_manager_->updateFloatingBaseDesired(
          sp_->curr_time);
      if ((sp_->curr_time >=
           floating_base_lifting_up_manager_->start_time_ +
               floating_base_lifting_up_manager_->duration_) &&
          (!floating_base_lifting_up_manager_->is_sinusoid)) {
        floating_base_lifting_up_manager_->is_swaying = false;
        std::cout << "Swaying Done" << std::endl;
      }
    }
    main_controller_->getCommand(_command); 
  }*/

  /*// Smoothing trq for initial state
  smoothing_torque(_command);
  // Save Data
  saveData();*/

  // Check for State Transitions
  if (state_machines_[state_]->endOfState()) {
    state_machines_[state_]->lastVisit();
    prev_state_ = state_;
    state_ = state_machines_[state_]->getNextState();
    b_state_first_visit_ = true;
  }
}

/*void DracoControlArchitecture::smoothing_torque(void* _cmd) {
  if (state_ == DRACO_STATES::INITIALIZE) {
    double rat = ((Initialize*)state_machines_[state_])->progression_variable();
    for (int i = 0; i < Draco::n_adof; ++i) {
      ((DracoCommand*)_cmd)->jtrq[i] =
          myUtils::smoothing(0, ((DracoCommand*)_cmd)->jtrq[i], rat);
      sp_->prev_trq_cmd[i] = ((DracoCommand*)_cmd)->jtrq[i];
    }
  }
  if (state_ == DRACO_STATES::STAND) {
    double rat =
        ((DoubleSupportStand*)state_machines_[state_])->progression_variable();
    for (int i = 0; i < Draco::n_adof; ++i) {
      ((DracoCommand*)_cmd)->jtrq[i] = myUtils::smoothing(
          sp_->prev_trq_cmd[i], ((DracoCommand*)_cmd)->jtrq[i], rat);
    }
  }
}

void DracoControlArchitecture::getIVDCommand(void* _cmd) {
  Eigen::VectorXd tau_cmd = Eigen::VectorXd::Zero(Draco::n_adof);
  Eigen::VectorXd des_jpos = Eigen::VectorXd::Zero(Draco::n_adof);
  Eigen::VectorXd des_jvel = Eigen::VectorXd::Zero(Draco::n_adof);

  Eigen::MatrixXd A = robot_->getMassMatrix();
  Eigen::VectorXd grav = robot_->getGravity();
  Eigen::VectorXd cori = robot_->getCoriolis();

  Eigen::VectorXd xddot_des = Eigen::VectorXd::Zero(Draco::n_adof);
  taf_container_->joint_task_->updateJacobians();
  taf_container_->joint_task_->computeCommands();
  taf_container_->joint_task_->getCommand(xddot_des);

  Eigen::VectorXd qddot_des = Eigen::VectorXd::Zero(Draco::n_dof);
  qddot_des.tail(Draco::n_adof) = xddot_des;

  des_jpos = sp_->q.tail(Draco::n_adof);
  des_jvel = sp_->qdot.tail(Draco::n_adof);
  tau_cmd = (A * qddot_des + cori + grav).tail(Draco::n_adof);

  for (int i(0); i < Draco::n_adof; ++i) {
    ((DracoCommand*)_cmd)->jtrq[i] = tau_cmd[i];
    ((DracoCommand*)_cmd)->q[i] = des_jpos[i];
    ((DracoCommand*)_cmd)->qdot[i] = des_jvel[i];
  }
}*/

void A1ControlArchitecture::_InitializeParameters() {
  // Controller initialization
  // main_controller_->ctrlInitialization(cfg_["controller_parameters"]);

  // Trajectory Managers initialization
  // Draco pulls these values from the yaml file in Config/A1/TEST
  // Here we just define following Valkyrie for simplicity
  try { 
    double max_gain, min_gain;
    myUtils::readParameter(cfg_["task_parameters"], "max_w_task_com", max_gain)
    myUtils::readParameter(cfg_["task_parameters"], "min_w_task_com", min_gain);

    flfoot_max_normal_force_manager_->setMaxFz(500.);
    rlfoot_max_normal_force_manager_->setMaxFz(500.);
    frfoot_max_normal_force_manager_->setMaxFz(500.);
    rrfoot_max_normal_force_manager_->setMaxFz(500.);
    flfoot_pos_hierarchy_manager_->setMaxGain(40.0);
    flfoot_pos_hierarchy_manager_->setMinGain(20.0);
    frfoot_pos_hierarchy_manager_->setMaxGain(40.0);
    frfoot_pos_hierarchy_manager_->setMinGain(20.0);
    rlfoot_pos_hierarchy_manager_->setMaxGain(40.0);
    rlfoot_pos_hierarchy_manager_->setMinGain(20.0);
    rrfoot_pos_hierarchy_manager_->setMaxGain(40.0);
    rrfoot_pos_hierarchy_manager_->setMinGain(20.0);
    com_hierarchy_manager_->setMaxGain(max_gain);
    com_hierarchy_manager_->setMinGain(min_gain);
    base_ori_hierarchy_manager_->setMaxGain(max_gain);
    base_ori_hierarchy_manager_->setMinGain(min_gain);
  } catch (std::runtime_error& e) {
      std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                << __FILE__ << "]" << std::endl
                << std::endl;
      exit(0);
  }
 
  // States Initialization:
  state_machines_[A1_STATES::INITIALIZE]->initialization(
      cfg_["state_initialize_params"]);
  // state_machines_[DRACO_STATES::STAND]->initialization(
  //     cfg_["state_stand_params"]);
  // state_machines_[DRACO_STATES::RL_SWING]->initialization(cfg_["state_swing"]);
  // state_machines_[DRACO_STATES::LL_SWING]->initialization(cfg_["state_swing"]);
}

/*void DracoControlArchitecture::saveData() {
  // Task weights, Reaction force weights
  sp_->w_rfoot_pos = rfoot_pos_hierarchy_manager_->current_w_;
  sp_->w_rfoot_ori = rfoot_ori_hierarchy_manager_->current_w_;
  sp_->w_lfoot_pos = lfoot_pos_hierarchy_manager_->current_w_;
  sp_->w_lfoot_ori = lfoot_ori_hierarchy_manager_->current_w_;
  sp_->w_com = com_hierarchy_manager_->current_w_;
  sp_->w_base_ori = base_ori_hierarchy_manager_->current_w_;
  //  sp_->w_rf_rffront =
  // rfoot_front_max_normal_force_manager_->current_max_normal_force_z_;
  // sp_->w_rf_rfback =
  // rfoot_back_max_normal_force_manager_->current_max_normal_force_z_;
  // sp_->w_rf_lffront =
  // lfoot_front_max_normal_force_manager_->current_max_normal_force_z_;
  // sp_->w_rf_lfback =
  //lfoot_back_max_normal_force_manager_->current_max_normal_force_z_;
  sp_->w_rfoot_fr =
      rfoot_max_normal_force_manager_->current_max_normal_force_z_;
  sp_->w_lfoot_fr =
      lfoot_max_normal_force_manager_->current_max_normal_force_z_;

  // Task desired
  sp_->rfoot_center_pos_des = rfoot_trajectory_manager_->foot_pos_des_;
  sp_->rfoot_center_vel_des = rfoot_trajectory_manager_->foot_vel_des_;
  sp_->lfoot_center_pos_des = lfoot_trajectory_manager_->foot_pos_des_;
  sp_->lfoot_center_vel_des = lfoot_trajectory_manager_->foot_vel_des_;
  sp_->rfoot_center_quat_des = rfoot_trajectory_manager_->foot_quat_des_;
  sp_->rfoot_center_so3_des = rfoot_trajectory_manager_->foot_ang_vel_des_;
  sp_->lfoot_center_quat_des = lfoot_trajectory_manager_->foot_quat_des_;
  sp_->lfoot_center_so3_des = lfoot_trajectory_manager_->foot_ang_vel_des_;
  sp_->q_task_des = joint_trajectory_manager_->joint_pos_des_;
  sp_->qdot_task_des = joint_trajectory_manager_->joint_vel_des_;
  sp_->q_task = sp_->q.tail(Draco::n_adof);
  sp_->qdot_task = sp_->qdot.tail(Draco::n_adof);

  if (state_ == DRACO_STATES::INITIALIZE || state_ == DRACO_STATES::STAND ||
      prev_state_ == DRACO_STATES::STAND) {
    sp_->com_pos_des = floating_base_lifting_up_manager_->com_pos_des_;
    sp_->com_vel_des = floating_base_lifting_up_manager_->com_vel_des_;
    sp_->dcm_des = floating_base_lifting_up_manager_->dcm_pos_des_;
    sp_->dcm_vel_des = floating_base_lifting_up_manager_->dcm_vel_des_;
    sp_->base_quat_des = floating_base_lifting_up_manager_->base_ori_quat_des_;
    sp_->base_ang_vel_des =
        floating_base_lifting_up_manager_->base_ang_vel_des_;
  } else {
    sp_->com_pos_des = dcm_trajectory_manager_->des_com_pos;
    sp_->com_vel_des = dcm_trajectory_manager_->des_com_vel;
    sp_->base_quat_des = dcm_trajectory_manager_->des_quat;
    sp_->base_ang_vel_des = dcm_trajectory_manager_->des_ang_vel;
    sp_->dcm_des = dcm_trajectory_manager_->des_dcm;
    sp_->dcm_vel_des = dcm_trajectory_manager_->des_dcm_vel;
  }
}*/
