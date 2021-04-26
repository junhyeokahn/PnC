#include <PnC/A1PnC/A1CtrlArchitecture/A1CtrlArchitecture.hpp>

A1ControlArchitecture::A1ControlArchitecture(RobotSystem* _robot)
    : ControlArchitecture(_robot) {
  b_state_first_visit_ = true;

  myUtils::pretty_constructor(1, "A1 Control Architecture");
  cfg_ = YAML::LoadFile(THIS_COM
                        "Config/A1/TEST/CONTROL_ARCHITECTURE_PARAMS.yaml");

  sp_ = A1StateProvider::getStateProvider(robot_);
  // Initialize Main Controller
  taf_container_ = new A1TaskAndForceContainer(robot_);
  taf_container_->paramInitialization(cfg_["task_parameters"]);
  main_controller_ = new A1MainController(taf_container_, robot_);

  // Initialize Planner
  body_inertia = Eigen::VectorXd::Zero(9);
  _MPC_WEIGHTS = Eigen::VectorXd::Zero(13);
  body_inertia[0] = 0.02; body_inertia[4] = 0.06; body_inertia[8] = 0.07;
  _MPC_WEIGHTS[0] = 5.; _MPC_WEIGHTS[1] = 5.; _MPC_WEIGHTS[2] = 0.2;
  _MPC_WEIGHTS[5] = 10.; _MPC_WEIGHTS[6] = 0.5; _MPC_WEIGHTS[7] = 0.5;
  _MPC_WEIGHTS[8] = 0.2; _MPC_WEIGHTS[9] = 0.2; _MPC_WEIGHTS[10] = 0.2;
  _MPC_WEIGHTS[11] = 0.1; 
  mpc_planner_ = new ConvexMPC(mass, body_inertia, num_legs,
                               _PLANNING_HORIZON_STEPS,
                               _PLANNING_TIMESTEP,
                               _MPC_WEIGHTS);
  // Initialize Trajectory managers
  floating_base_lifting_up_manager_ = new FloatingBaseTrajectoryManager(
      taf_container_->com_task_, taf_container_->base_ori_task_, robot_);
  frfoot_trajectory_manager_ =
      new PointFootTrajectoryManager(taf_container_->frfoot_pos_task_, robot_);
  flfoot_trajectory_manager_ =
      new PointFootTrajectoryManager(taf_container_->flfoot_pos_task_, robot_);
  rrfoot_trajectory_manager_ =
      new PointFootTrajectoryManager(taf_container_->rrfoot_pos_task_, robot_);
  rlfoot_trajectory_manager_ =
      new PointFootTrajectoryManager(taf_container_->rlfoot_pos_task_, robot_);

  frfoot_max_normal_force_manager_ = new MaxNormalForceTrajectoryManager(
      taf_container_->frfoot_contact_, robot_);
  rrfoot_max_normal_force_manager_ = new MaxNormalForceTrajectoryManager(
      taf_container_->rrfoot_contact_, robot_);
  flfoot_max_normal_force_manager_ = new MaxNormalForceTrajectoryManager(
      taf_container_->flfoot_contact_, robot_);
  rlfoot_max_normal_force_manager_ = new MaxNormalForceTrajectoryManager(
      taf_container_->rlfoot_contact_, robot_);

  frfoot_pos_hierarchy_manager_ =
      new TaskWeightTrajectoryManager(taf_container_->frfoot_pos_task_, robot_);
  rrfoot_pos_hierarchy_manager_ =
      new TaskWeightTrajectoryManager(taf_container_->rrfoot_pos_task_, robot_);
  flfoot_pos_hierarchy_manager_ =
      new TaskWeightTrajectoryManager(taf_container_->flfoot_pos_task_, robot_);
  rlfoot_pos_hierarchy_manager_ =
      new TaskWeightTrajectoryManager(taf_container_->rlfoot_pos_task_, robot_);

  com_hierarchy_manager_ =
      new TaskWeightTrajectoryManager(taf_container_->com_task_, robot_);
  base_ori_hierarchy_manager_ =
      new TaskWeightTrajectoryManager(taf_container_->base_ori_task_, robot_);

  rxn_force_manager_ =
      new ReactionForceTrajectoryManager(_PLANNING_TIMESTEP, _PLANNING_HORIZON_STEPS, robot_);

  // Initialize states: add all states to the state machine map
  state_machines_[A1_STATES::STAND] =
      new QuadSupportStand(A1_STATES::STAND, this, robot_);
  state_machines_[A1_STATES::BALANCE] =
      new QuadSupportBalance(A1_STATES::BALANCE, this, robot_);
  state_machines_[A1_STATES::FL_CONTACT_TRANSITION_START] =
      new ContactTransitionStart(A1_STATES::FL_CONTACT_TRANSITION_START,
                                 LEFT_ROBOT_SIDE, this, robot_);
  state_machines_[A1_STATES::FL_CONTACT_TRANSITION_END] =
      new ContactTransitionEnd(A1_STATES::FL_CONTACT_TRANSITION_END,
                               LEFT_ROBOT_SIDE, this, robot_);
  state_machines_[A1_STATES::FL_SWING] =
      new SwingControl(A1_STATES::FL_SWING, LEFT_ROBOT_SIDE, this, robot_);
  state_machines_[A1_STATES::FR_CONTACT_TRANSITION_START] =
      new ContactTransitionStart(A1_STATES::FR_CONTACT_TRANSITION_START,
                                 RIGHT_ROBOT_SIDE, this, robot_);
  state_machines_[A1_STATES::FR_CONTACT_TRANSITION_END] =
      new ContactTransitionEnd(A1_STATES::FR_CONTACT_TRANSITION_END,
                               RIGHT_ROBOT_SIDE, this, robot_);
  state_machines_[A1_STATES::FR_SWING] =
      new SwingControl(A1_STATES::FR_SWING, RIGHT_ROBOT_SIDE, this, robot_);

  // Initialize MPC Variables
  foot_contact_states = Eigen::VectorXi::Zero(4);
  foot_pos_body_frame = Eigen::VectorXd::Zero(12);
  foot_friction_coeffs = Eigen::VectorXd::Zero(4);
  flfoot_body_frame << 0., 0., 0.;
  frfoot_body_frame << 0., 0., 0.;
  rlfoot_body_frame << 0., 0., 0.;
  rrfoot_body_frame << 0., 0., 0.;
  // double temp = taf_container_->frfoot_contact_->mu_;
  foot_friction_coeffs << 0.3, 0.3, 0.3, 0.3;
  mpc_counter = 6;

  // Initialize variable for setting contact forces
  command_rxn_forces = Eigen::VectorXd::Zero(12);

  // Set Starting State
  state_ = A1_STATES::STAND;
  prev_state_ = state_;

  _InitializeParameters();
}

A1ControlArchitecture::~A1ControlArchitecture() {
  delete taf_container_;
  delete main_controller_;
  delete rxn_force_manager_;
  delete mpc_planner_;

  // Delete the trajectory managers
  delete frfoot_max_normal_force_manager_;
  delete rrfoot_max_normal_force_manager_;
  delete flfoot_max_normal_force_manager_;
  delete rlfoot_max_normal_force_manager_;
  // delete joint_trajectory_manager_;
  delete floating_base_lifting_up_manager_;

  delete frfoot_trajectory_manager_;
  delete flfoot_trajectory_manager_;
  delete rrfoot_trajectory_manager_;
  delete rlfoot_trajectory_manager_;

  delete frfoot_pos_hierarchy_manager_;
  delete rrfoot_pos_hierarchy_manager_;
  delete rlfoot_pos_hierarchy_manager_;
  delete flfoot_pos_hierarchy_manager_;
  delete com_hierarchy_manager_;
  delete base_ori_hierarchy_manager_;

  // Delete the state machines
  delete state_machines_[A1_STATES::STAND];
  delete state_machines_[A1_STATES::BALANCE];
  delete state_machines_[A1_STATES::FL_CONTACT_TRANSITION_START];
  delete state_machines_[A1_STATES::FL_CONTACT_TRANSITION_END];
  delete state_machines_[A1_STATES::FL_SWING];
  delete state_machines_[A1_STATES::FR_CONTACT_TRANSITION_START];
  delete state_machines_[A1_STATES::FR_CONTACT_TRANSITION_END];
  delete state_machines_[A1_STATES::FR_SWING];
}

void A1ControlArchitecture::ControlArchitectureInitialization() {}

void A1ControlArchitecture::solveMPC() {
    // Set contact state
    if(sp_->b_flfoot_contact) foot_contact_states[0] = 1;
    else foot_contact_states[0] = 0;
    if(sp_->b_frfoot_contact) foot_contact_states[1] = 1;
    else foot_contact_states[1] = 0;
    if(sp_->b_rlfoot_contact) foot_contact_states[2] = 1;
    else foot_contact_states[2] = 0;
    if(sp_->b_rrfoot_contact) foot_contact_states[3] = 1;
    else foot_contact_states[3] = 0;

    // std::cout << "foot_contact_states = " << foot_contact_states[0] << ", " << foot_contact_states[1] << ", " << foot_contact_states[2] << ", " << foot_contact_states[3] << std::endl;

    Eigen::Vector3d com_pos_des, com_pos;
    // CoM Position // TODO STATE ESTIMATOR
    com_pos[0] = 0; com_pos[1] = 0; com_pos[2] = sp_->com_pos[2];
    // CoM Desired Position (x,y plane not necessary)
    com_pos_des[0] = 0; com_pos_des[1] = 0; com_pos_des[2] = 0.3;
    // Current CoM Angular Velocity
    Eigen::Vector3d rpy_dot;
    rpy_dot = robot_->getBodyNodeCoMSpatialVelocity(A1BodyNode::trunk).head(3);
    // Desired rpy_dot
    Eigen::Vector3d rpy_des;
    rpy_des = Eigen::VectorXd::Zero(3);
    // Get Foot Positions Body Frame
    Eigen::Vector3d base_in_world =
        robot_->getBodyNodeIsometry(A1BodyNode::trunk).translation();
    Eigen::Vector3d temp_foot_world =
        robot_->getBodyNodeIsometry(A1BodyNode::FL_foot).translation();
    flfoot_body_frame = temp_foot_world - base_in_world;
    temp_foot_world =
        robot_->getBodyNodeIsometry(A1BodyNode::FR_foot).translation();
    frfoot_body_frame = temp_foot_world - base_in_world;
    temp_foot_world =
        robot_->getBodyNodeIsometry(A1BodyNode::RL_foot).translation();
    rlfoot_body_frame = temp_foot_world - base_in_world;
    temp_foot_world =
        robot_->getBodyNodeIsometry(A1BodyNode::RR_foot).translation();
    rrfoot_body_frame = temp_foot_world - base_in_world;
    foot_pos_body_frame << 
        flfoot_body_frame[0], flfoot_body_frame[1], flfoot_body_frame[2],
        frfoot_body_frame[0], frfoot_body_frame[1], frfoot_body_frame[2],
        rlfoot_body_frame[0], rlfoot_body_frame[1], rlfoot_body_frame[2],
        rrfoot_body_frame[0], rrfoot_body_frame[1], rrfoot_body_frame[2];

    // Get com_vel_body_frame
    Eigen::VectorXd com_vel_body_frame, com_vel_world_frame;
    com_vel_body_frame = Eigen::VectorXd::Zero(3);
    com_vel_world_frame = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd rot = robot_->getBodyNodeIsometry(A1BodyNode::trunk).linear();
    com_vel_body_frame = rot * com_vel_world_frame;
    // Get roll_pitch_yaw of robot CoM in ZYX
    Eigen::VectorXd com_rpy_zyx;
    com_rpy_zyx = Eigen::VectorXd::Zero(3);
    if((rot.row(0)[0] == rot.row(1)[0]) && rot.row(0)[0] <= 0.00001) {
      double rz = 0; double ry = M_PI / 2.; 
      double rx = atan2(rot.row(0)[1], rot.row(1)[1]);
      com_rpy_zyx[0] = rx; com_rpy_zyx[1] = ry; com_rpy_zyx[2] = rz; 
    } else {
      double rz = atan2(rot.row(1)[0], rot.row(0)[0]);
      double ry = atan2(-rot.row(2)[0], 
                        sqrt(rot.row(0)[0] + rot.row(1)[0]));
      double rx = atan2(rot.row(2)[1], rot.row(2)[2]);
      com_rpy_zyx[0] = rx; com_rpy_zyx[1] = ry; com_rpy_zyx[2] = rz;
    }

    // Feed a zero vector as com_pos --> use height estimator inside MPC
    Eigen::VectorXd tmp; tmp = Eigen::VectorXd::Zero(1);

    sp_->mpc_rxn_forces = mpc_planner_->ComputeContactForces(
        com_pos,//tmp,// com_pos, // com_pos
        com_vel_body_frame, // com_vel_body_frame
        com_rpy_zyx, // RPY in the ZYX sense
        rpy_dot, //com_ang_vel
        foot_contact_states,  // foot contact_states
        foot_pos_body_frame, //foot_pos_body_frame
        foot_friction_coeffs, //foot_friction_coeffs
        com_pos_des, // com_pos_des
        sp_->x_y_yaw_vel_des, //com_vel_des
        rpy_des, // rpy_des
        sp_->x_y_yaw_vel_des); // com ang vel des

    // myUtils::pretty_print(sp_->mpc_rxn_forces, std::cout, "MPC Rxn Forces");
}

void A1ControlArchitecture::getCommand(void* _command) {
  // Initialize State
  if (b_state_first_visit_) {
    state_machines_[state_]->firstVisit();
    b_state_first_visit_ = false;
  }
  //if(state_ != A1_STATES::BALANCE && state_ != A1_STATES::STAND){
    if(mpc_counter >= 6){// Call the MPC at 83.3 Hz
        solveMPC();
        if(sp_->mpc_rxn_forces.size() > 10){
          rxn_force_manager_->updateSolution(
                        sp_->curr_time,
                        sp_->mpc_rxn_forces);
          mpc_counter = 0;
        }
    } else {++mpc_counter;}
    // Get the interpolated value for reaction forces from previous MPC call
    command_rxn_forces = rxn_force_manager_->getRFSolution(sp_->curr_time);
    // myUtils::pretty_print(command_rxn_forces,std::cout, "interpolated RXN forces");
    // Set the Contact Level Rxn Forces
    Eigen::VectorXd tmp_rxn_forces; tmp_rxn_forces = Eigen::VectorXd::Zero(3);
    tmp_rxn_forces[0] = command_rxn_forces[0];
    tmp_rxn_forces[1] = command_rxn_forces[1];
    tmp_rxn_forces[2] = -command_rxn_forces[2];
    taf_container_->flfoot_contact_->setRFDesired(tmp_rxn_forces);
    tmp_rxn_forces[0] = command_rxn_forces[3];
    tmp_rxn_forces[1] = command_rxn_forces[4];
    tmp_rxn_forces[2] = -command_rxn_forces[5];
    taf_container_->frfoot_contact_->setRFDesired(tmp_rxn_forces);
    tmp_rxn_forces[0] = command_rxn_forces[6];
    tmp_rxn_forces[1] = command_rxn_forces[7];
    tmp_rxn_forces[2] = -command_rxn_forces[8];
    taf_container_->rlfoot_contact_->setRFDesired(tmp_rxn_forces);
    tmp_rxn_forces[0] = command_rxn_forces[9];
    tmp_rxn_forces[1] = command_rxn_forces[10];
    tmp_rxn_forces[2] = -command_rxn_forces[11];
    taf_container_->rrfoot_contact_->setRFDesired(tmp_rxn_forces);
  // }
  // Update State Machine
  state_machines_[state_]->oneStep();
  // Swaying
  if(state_ == A1_STATES::BALANCE &&
      (floating_base_lifting_up_manager_->is_swaying ||
      floating_base_lifting_up_manager_->is_sinusoid)){
    floating_base_lifting_up_manager_->updateFloatingBaseDesired(sp_->curr_time);
    if((sp_->curr_time >= floating_base_lifting_up_manager_->start_time_ +
                floating_base_lifting_up_manager_->duration_) && 
                (!floating_base_lifting_up_manager_->is_sinusoid)) {
        floating_base_lifting_up_manager_->is_swaying = false;
        std::cout << "Swaying Done" << std::endl;
    }
  }
  // Set boolean to determine if we want to change QP weights
  bool change_qp_weights_for_walking = true;
  /*if(state_ != A1_STATES::BALANCE || state_ != A1_STATES::STAND) {
    change_qp_weights_for_walking = true;
  } else { change_qp_weights_for_walking = false; }*/
  // std::cout << "change_qp_weights_for_walking = " << change_qp_weights_for_walking << std::endl;
  // Get Wholebody control commands
  main_controller_->getCommand(_command, change_qp_weights_for_walking);
  // Save Data
  saveData();

  // Check for State Transitions
  if (state_machines_[state_]->endOfState()) {
    state_machines_[state_]->lastVisit();
    prev_state_ = state_;
    state_ = state_machines_[state_]->getNextState();
    b_state_first_visit_ = true;
  }
}


void A1ControlArchitecture::_InitializeParameters() {
  // Controller initialization
  main_controller_->ctrlInitialization(cfg_["controller_parameters"]);

  // Trajectory Managers initialization
  try {
    double max_gain, min_gain, max_fz;
    myUtils::readParameter(cfg_["task_parameters"], "max_w_task_com", max_gain);
    myUtils::readParameter(cfg_["task_parameters"], "min_w_task_com", min_gain);
    myUtils::readParameter(cfg_["task_parameters"], "max_z_force", max_fz);

    flfoot_max_normal_force_manager_->setMaxFz(max_fz);
    rlfoot_max_normal_force_manager_->setMaxFz(max_fz);
    frfoot_max_normal_force_manager_->setMaxFz(max_fz);
    rrfoot_max_normal_force_manager_->setMaxFz(max_fz);
    flfoot_pos_hierarchy_manager_->setMaxGain(40.0);
    flfoot_pos_hierarchy_manager_->setMinGain(20.0);
    frfoot_pos_hierarchy_manager_->setMaxGain(40.0);
    frfoot_pos_hierarchy_manager_->setMinGain(20.0);
    rlfoot_pos_hierarchy_manager_->setMaxGain(40.0);
    rlfoot_pos_hierarchy_manager_->setMinGain(20.0);
    rrfoot_pos_hierarchy_manager_->setMaxGain(40.0);
    rrfoot_pos_hierarchy_manager_->setMinGain(20.0);
    com_hierarchy_manager_->setMaxGain(400.);
    com_hierarchy_manager_->setMinGain(20.);
    base_ori_hierarchy_manager_->setMaxGain(200.);
    base_ori_hierarchy_manager_->setMinGain(20.);
  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }

  // States Initialization:
  state_machines_[A1_STATES::STAND]->initialization(cfg_["state_stand_params"]);
  state_machines_[A1_STATES::FR_SWING]->initialization(cfg_["state_swing"]);
  state_machines_[A1_STATES::FL_SWING]->initialization(cfg_["state_swing"]);
  state_machines_[A1_STATES::FL_CONTACT_TRANSITION_START]->initialization(cfg_["state_contact_transition"]);
  state_machines_[A1_STATES::FL_CONTACT_TRANSITION_END]->initialization(cfg_["state_contact_transition"]);
  state_machines_[A1_STATES::FR_CONTACT_TRANSITION_START]->initialization(cfg_["state_contact_transition"]);
  state_machines_[A1_STATES::FR_CONTACT_TRANSITION_END]->initialization(cfg_["state_contact_transition"]);
}

void A1ControlArchitecture::saveData() {
  // Task weights, Reaction force weights
  sp_->w_frfoot_pos = frfoot_pos_hierarchy_manager_->current_w_;
  sp_->w_rrfoot_pos = rrfoot_pos_hierarchy_manager_->current_w_;
  sp_->w_flfoot_pos = flfoot_pos_hierarchy_manager_->current_w_;
  sp_->w_rlfoot_pos = rlfoot_pos_hierarchy_manager_->current_w_;
  sp_->w_com = com_hierarchy_manager_->current_w_;
  sp_->w_base_ori = base_ori_hierarchy_manager_->current_w_;
  sp_->w_frfoot_fr =
      frfoot_max_normal_force_manager_->current_max_normal_force_z_;
  sp_->w_flfoot_fr =
      flfoot_max_normal_force_manager_->current_max_normal_force_z_;
  sp_->w_rrfoot_fr =
      rrfoot_max_normal_force_manager_->current_max_normal_force_z_;
  sp_->w_rlfoot_fr =
      rlfoot_max_normal_force_manager_->current_max_normal_force_z_;

  sp_->interpolated_mpc_forces = command_rxn_forces;

  // Task desired
  // sp_->q_task_des = joint_trajectory_manager_->joint_pos_des_;
  // sp_->qdot_task_des = joint_trajectory_manager_->joint_vel_des_;
  sp_->q_task = sp_->q.tail(A1::n_adof);
  sp_->qdot_task = sp_->qdot.tail(A1::n_adof);

  sp_->com_pos_des = floating_base_lifting_up_manager_->com_pos_des_;
  sp_->com_vel_des = floating_base_lifting_up_manager_->com_vel_des_;
  sp_->base_quat_des = floating_base_lifting_up_manager_->base_ori_quat_des_;
  sp_->base_ang_vel_des = floating_base_lifting_up_manager_->base_ang_vel_des_;

  sp_->frfoot_pos_des = frfoot_trajectory_manager_->foot_pos_des_;
  sp_->flfoot_pos_des = flfoot_trajectory_manager_->foot_pos_des_;
  sp_->rrfoot_pos_des = rrfoot_trajectory_manager_->foot_pos_des_;
  sp_->rlfoot_pos_des = rlfoot_trajectory_manager_->foot_pos_des_;

  sp_->frfoot_vel_des = frfoot_trajectory_manager_->foot_vel_des_;
  sp_->flfoot_vel_des = flfoot_trajectory_manager_->foot_vel_des_;
  sp_->rrfoot_vel_des = rrfoot_trajectory_manager_->foot_vel_des_;
  sp_->rlfoot_vel_des = rlfoot_trajectory_manager_->foot_vel_des_;
}
