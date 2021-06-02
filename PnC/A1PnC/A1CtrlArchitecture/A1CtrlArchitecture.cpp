#include <PnC/A1PnC/A1CtrlArchitecture/A1CtrlArchitecture.hpp>
#include <sys/time.h>

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
  body_inertia[0] = 0.01; body_inertia[4] = 0.03; body_inertia[8] = 0.03;

  // Stepping in place infinitely weights
  _MPC_WEIGHTS[0] = 5.; _MPC_WEIGHTS[1] = 5.; _MPC_WEIGHTS[2] = 0.2;
  _MPC_WEIGHTS[5] = 15.; _MPC_WEIGHTS[6] = 0.5; _MPC_WEIGHTS[7] = 0.5;
  _MPC_WEIGHTS[8] = 0.2; _MPC_WEIGHTS[9] = 10.; _MPC_WEIGHTS[10] = 5.;
  _MPC_WEIGHTS[11] = 0.1; 
  /*// Weights from Sangbae
  _MPC_WEIGHTS[0] = 1.; _MPC_WEIGHTS[1] = 1.; _MPC_WEIGHTS[2] = 1.; _MPC_WEIGHTS[3] = 0.;
  _MPC_WEIGHTS[4] = 0.; _MPC_WEIGHTS[5] = 50.; _MPC_WEIGHTS[6] = 0; _MPC_WEIGHTS[7] = 0;
  _MPC_WEIGHTS[8] = 1; _MPC_WEIGHTS[9] = 1; _MPC_WEIGHTS[10] = 1; _MPC_WEIGHTS[11] = 1;
  _MPC_WEIGHTS[12] = 0.;*/
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

  state_machines_[A1_STATES::FL_HALF_SWING] =
      new SwingControl(A1_STATES::FL_HALF_SWING, LEFT_ROBOT_SIDE, this, robot_);
  state_machines_[A1_STATES::FL_FINAL_SWING] =
      new SwingControl(A1_STATES::FL_FINAL_SWING, LEFT_ROBOT_SIDE, this, robot_);

  state_machines_[A1_STATES::FR_CONTACT_TRANSITION_START] =
      new ContactTransitionStart(A1_STATES::FR_CONTACT_TRANSITION_START,
                                 RIGHT_ROBOT_SIDE, this, robot_);
  state_machines_[A1_STATES::FR_CONTACT_TRANSITION_END] =
      new ContactTransitionEnd(A1_STATES::FR_CONTACT_TRANSITION_END,
                               RIGHT_ROBOT_SIDE, this, robot_);

  state_machines_[A1_STATES::FR_HALF_SWING] =
      new SwingControl(A1_STATES::FR_HALF_SWING, RIGHT_ROBOT_SIDE, this, robot_);
  state_machines_[A1_STATES::FR_FINAL_SWING] =
      new SwingControl(A1_STATES::FR_FINAL_SWING, RIGHT_ROBOT_SIDE, this, robot_);


  // Initialize MPC Variables
  foot_contact_states = Eigen::VectorXi::Zero(4);
  foot_pos_body_frame = Eigen::VectorXd::Zero(12);
  foot_friction_coeffs = Eigen::VectorXd::Zero(4);
  flfoot_body_frame << 0., 0., 0.;
  frfoot_body_frame << 0., 0., 0.;
  rlfoot_body_frame << 0., 0., 0.;
  rrfoot_body_frame << 0., 0., 0.;
  // double temp = taf_container_->frfoot_contact_->mu_;
  foot_friction_coeffs << 0.6, 0.6, 0.6, 0.6;
  mpc_counter = 6;
  num_mpc_calls = 0;

  // Initialize variable for setting contact forces
  command_rxn_forces = Eigen::VectorXd::Zero(12);

  // Set Starting State
  state_ = A1_STATES::STAND;
  prev_state_ = state_;
  state_holder_for_mpc_ = state_;

  test_counter = 0;
  sp_->flfoot_landing = robot_->getBodyNodeIsometry(A1BodyNode::FL_foot).translation();
  sp_->frfoot_landing = robot_->getBodyNodeIsometry(A1BodyNode::FR_foot).translation();
  sp_->rlfoot_landing = robot_->getBodyNodeIsometry(A1BodyNode::RL_foot).translation();
  sp_->rrfoot_landing = robot_->getBodyNodeIsometry(A1BodyNode::RR_foot).translation();

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
  delete state_machines_[A1_STATES::FL_HALF_SWING];
  delete state_machines_[A1_STATES::FL_FINAL_SWING];
  delete state_machines_[A1_STATES::FR_CONTACT_TRANSITION_START];
  delete state_machines_[A1_STATES::FR_CONTACT_TRANSITION_END];
  delete state_machines_[A1_STATES::FR_HALF_SWING];
  delete state_machines_[A1_STATES::FR_FINAL_SWING];
}

void A1ControlArchitecture::ControlArchitectureInitialization() {}

void A1ControlArchitecture::solveMPC() {
    /*// Set contact state
    if(sp_->b_flfoot_contact) foot_contact_states[0] = 1;
    else foot_contact_states[0] = 0;
    if(sp_->b_frfoot_contact) foot_contact_states[1] = 1;
    else foot_contact_states[1] = 0;
    if(sp_->b_rlfoot_contact) foot_contact_states[2] = 1;
    else foot_contact_states[2] = 0;
    if(sp_->b_rrfoot_contact) foot_contact_states[3] = 1;
    else foot_contact_states[3] = 0;*/
    if(state_ == A1_STATES::BALANCE || state_ == A1_STATES::STAND ||
       state_ == A1_STATES::FR_CONTACT_TRANSITION_START ||
       state_ == A1_STATES::FL_CONTACT_TRANSITION_START) {
        foot_contact_states[0] = 1;
        foot_contact_states[1] = 1;
        foot_contact_states[2] = 1;
        foot_contact_states[3] = 1;
    }
    if(state_ == A1_STATES::FL_HALF_SWING || state_ == A1_STATES::FL_FINAL_SWING ||
       state_ == A1_STATES::FL_CONTACT_TRANSITION_END) {
        foot_contact_states[0] = 0;
        foot_contact_states[1] = 1;
        foot_contact_states[2] = 1;
        foot_contact_states[3] = 0;
    }
    if(state_ == A1_STATES::FR_HALF_SWING || state_ == A1_STATES::FR_FINAL_SWING ||
       state_ == A1_STATES::FR_CONTACT_TRANSITION_END) {
        foot_contact_states[0] = 1;
        foot_contact_states[1] = 0;
        foot_contact_states[2] = 0;
        foot_contact_states[3] = 1;
    }
    // std::cout << "foot_contact_states = " << foot_contact_states[0] << ", " << foot_contact_states[1] << ", " << foot_contact_states[2] << ", " << foot_contact_states[3] << std::endl;

    Eigen::Vector3d com_pos_des, com_pos;
    // CoM Position // TODO STATE ESTIMATOR
    com_pos[0] = 0; com_pos[1] = 0; com_pos[2] = sp_->com_pos[2];
    // CoM Desired Position (x,y plane not necessary)
    com_pos_des[0] = 0; com_pos_des[1] = 0; com_pos_des[2] = sp_->com_pos_des[2];
    // Current CoM Angular Velocity
    Eigen::Vector3d ang_vel;
    ang_vel = robot_->getBodyNodeCoMSpatialVelocity(A1BodyNode::trunk).head(3);
    // Desired rpy_dot
    Eigen::Vector3d ang_vel_des;
    ang_vel_des = Eigen::VectorXd::Zero(3);
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
    if((rot.row(1)[0] <= 0.0001) && rot.row(0)[0] <= 0.0001) {
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
    // Gather the state transition of the MPC
    Eigen::VectorXd state_progression_, des_states_;
    state_progression_ = Eigen::VectorXd::Zero(13 * _PLANNING_HORIZON_STEPS);
    des_states_ = Eigen::VectorXd::Zero(13 * _PLANNING_HORIZON_STEPS);

    sp_->mpc_rxn_forces = mpc_planner_->ComputeContactForces(
        com_pos,//tmp,// com_pos, // com_pos
        com_vel_body_frame, // com_vel_body_frame
        com_rpy_zyx, // RPY in the ZYX sense
        ang_vel, //com_ang_vel
        foot_contact_states,  // foot contact_states
        foot_pos_body_frame, //foot_pos_body_frame
        foot_friction_coeffs, //foot_friction_coeffs
        com_pos_des, // com_pos_des
        sp_->x_y_yaw_vel_des, //com_vel_des
        ang_vel_des, // ang_vel_des
        sp_->x_y_yaw_vel_des, // com ang vel des
        state_progression_, // track the internal MPC state progression
        des_states_ ); // track the internal MPC desired state progression
    saveMPCSolution(com_pos, com_vel_body_frame, com_rpy_zyx, ang_vel, foot_pos_body_frame,
                   state_progression_, des_states_);
    // std::cout << "sp_->mpc_rxn_forces.size() = " << sp_->mpc_rxn_forces.size() << std::endl;

    ++num_mpc_calls;
    // myUtils::pretty_print(sp_->mpc_rxn_forces, std::cout, "MPC Rxn Forces");
}

void A1ControlArchitecture::saveMPCSolution(const Eigen::VectorXd &com_pos,
                                            const Eigen::VectorXd &com_vel_body_frame,
                                            const Eigen::VectorXd &com_rpy_zyx,
                                            const Eigen::VectorXd &ang_vel,
                                            const Eigen::VectorXd &foot_pos_body_frame,
                                            const Eigen::VectorXd &state_progression_,
                                            const Eigen::VectorXd &des_states_) {
  try {
    double t_start = sp_->curr_time;
    double t_end = t_start + _PLANNING_HORIZON_STEPS * _PLANNING_TIMESTEP;
    double t_step = _PLANNING_TIMESTEP;

    YAML::Node cfg;

    // =====================================================================
    // Temporal Parameters
    // =====================================================================

    cfg["temporal_parameters"]["initial_time"] = t_start;
    cfg["temporal_parameters"]["final_time"] = t_end;
    cfg["temporal_parameters"]["time_step"] = t_step;

    // =====================================================================
    // Input Information
    // =====================================================================

    cfg["input"]["com_pos"] = com_pos;
    cfg["input"]["com_vel_body_frame"] = com_vel_body_frame;
    cfg["input"]["com_rpy_zyx"] = com_rpy_zyx;
    cfg["input"]["ang_vel"] = ang_vel;

    Eigen::VectorXd tmp;
    tmp = Eigen::VectorXd::Zero(3);

    // x_y_yaw_vel_des
    tmp[0] = sp_->x_y_yaw_vel_des[0]; tmp[1] = sp_->x_y_yaw_vel_des[1];
    cfg["input"]["com_vel_des"] = tmp;
    tmp = Eigen::VectorXd::Zero(3);
    tmp[2] = sp_->x_y_yaw_vel_des[2];
    cfg["input"]["yaw_vel_des"] = tmp;
    // foot pos body frame
    tmp[0] = foot_pos_body_frame[0]; tmp[1] = foot_pos_body_frame[1]; tmp[2] = foot_pos_body_frame[2];
    cfg["input"]["flfoot_pos_body_frame"] = tmp;
    tmp[0] = foot_pos_body_frame[3]; tmp[1] = foot_pos_body_frame[4]; tmp[2] = foot_pos_body_frame[5];
    cfg["input"]["frfoot_pos_body_frame"] = tmp;
    tmp[0] = foot_pos_body_frame[6]; tmp[1] = foot_pos_body_frame[7]; tmp[2] = foot_pos_body_frame[8];
    cfg["input"]["rlfoot_pos_body_frame"] = tmp;
    tmp[0] = foot_pos_body_frame[9]; tmp[1] = foot_pos_body_frame[10]; tmp[2] = foot_pos_body_frame[11];
    cfg["input"]["rrfoot_pos_body_frame"] = tmp;

    // =====================================================================
    // Output Reaction Forces
    // =====================================================================

    Eigen::VectorXd flfoot_forces; flfoot_forces = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd frfoot_forces; frfoot_forces = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd rlfoot_forces; rlfoot_forces = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd rrfoot_forces; rrfoot_forces = Eigen::VectorXd::Zero(3);

    // Timestep 1
    flfoot_forces[0] = sp_->mpc_rxn_forces[0]; flfoot_forces[1] = sp_->mpc_rxn_forces[1]; flfoot_forces[2] = sp_->mpc_rxn_forces[2];
    frfoot_forces[0] = sp_->mpc_rxn_forces[3]; frfoot_forces[1] = sp_->mpc_rxn_forces[4]; frfoot_forces[2] = sp_->mpc_rxn_forces[5];
    rlfoot_forces[0] = sp_->mpc_rxn_forces[6]; rlfoot_forces[1] = sp_->mpc_rxn_forces[7]; rlfoot_forces[2] = sp_->mpc_rxn_forces[8];
    rrfoot_forces[0] = sp_->mpc_rxn_forces[9]; rrfoot_forces[1] = sp_->mpc_rxn_forces[10];rrfoot_forces[2] = sp_->mpc_rxn_forces[11];
    cfg["output"]["plan_time_1"]["flfoot_forces"] = flfoot_forces; cfg["output"]["plan_time_1"]["frfoot_forces"] = frfoot_forces;
    cfg["output"]["plan_time_1"]["rlfoot_forces"] = rlfoot_forces; cfg["output"]["plan_time_1"]["rrfoot_forces"] = rrfoot_forces;
    // Timestep 2
    flfoot_forces[0] = sp_->mpc_rxn_forces[12+0]; flfoot_forces[1] = sp_->mpc_rxn_forces[12+1]; flfoot_forces[2] = sp_->mpc_rxn_forces[12+2];
    frfoot_forces[0] = sp_->mpc_rxn_forces[12+3]; frfoot_forces[1] = sp_->mpc_rxn_forces[12+4]; frfoot_forces[2] = sp_->mpc_rxn_forces[12+5];
    rlfoot_forces[0] = sp_->mpc_rxn_forces[12+6]; rlfoot_forces[1] = sp_->mpc_rxn_forces[12+7]; rlfoot_forces[2] = sp_->mpc_rxn_forces[12+8];
    rrfoot_forces[0] = sp_->mpc_rxn_forces[12+9]; rrfoot_forces[1] = sp_->mpc_rxn_forces[12+10];rrfoot_forces[2] = sp_->mpc_rxn_forces[12+11];
    cfg["output"]["plan_time_2"]["flfoot_forces"] = flfoot_forces; cfg["output"]["plan_time_2"]["frfoot_forces"] = frfoot_forces;
    cfg["output"]["plan_time_2"]["rlfoot_forces"] = rlfoot_forces; cfg["output"]["plan_time_2"]["rrfoot_forces"] = rrfoot_forces;
    // Timestep 3
    flfoot_forces[0] = sp_->mpc_rxn_forces[24+0]; flfoot_forces[1] = sp_->mpc_rxn_forces[24+1]; flfoot_forces[2] = sp_->mpc_rxn_forces[24+2];
    frfoot_forces[0] = sp_->mpc_rxn_forces[24+3]; frfoot_forces[1] = sp_->mpc_rxn_forces[24+4]; frfoot_forces[2] = sp_->mpc_rxn_forces[24+5];
    rlfoot_forces[0] = sp_->mpc_rxn_forces[24+6]; rlfoot_forces[1] = sp_->mpc_rxn_forces[24+7]; rlfoot_forces[2] = sp_->mpc_rxn_forces[24+8];
    rrfoot_forces[0] = sp_->mpc_rxn_forces[24+9]; rrfoot_forces[1] = sp_->mpc_rxn_forces[24+10];rrfoot_forces[2] = sp_->mpc_rxn_forces[24+11];
    cfg["output"]["plan_time_3"]["flfoot_forces"] = flfoot_forces; cfg["output"]["plan_time_3"]["frfoot_forces"] = frfoot_forces;
    cfg["output"]["plan_time_3"]["rlfoot_forces"] = rlfoot_forces; cfg["output"]["plan_time_3"]["rrfoot_forces"] = rrfoot_forces;
    // Timestep 4
    flfoot_forces[0] = sp_->mpc_rxn_forces[36+0]; flfoot_forces[1] = sp_->mpc_rxn_forces[36+1]; flfoot_forces[2] = sp_->mpc_rxn_forces[36+2];
    frfoot_forces[0] = sp_->mpc_rxn_forces[36+3]; frfoot_forces[1] = sp_->mpc_rxn_forces[36+4]; frfoot_forces[2] = sp_->mpc_rxn_forces[36+5];
    rlfoot_forces[0] = sp_->mpc_rxn_forces[36+6]; rlfoot_forces[1] = sp_->mpc_rxn_forces[36+7]; rlfoot_forces[2] = sp_->mpc_rxn_forces[36+8];
    rrfoot_forces[0] = sp_->mpc_rxn_forces[36+9]; rrfoot_forces[1] = sp_->mpc_rxn_forces[36+10];rrfoot_forces[2] = sp_->mpc_rxn_forces[36+11];
    cfg["output"]["plan_time_4"]["flfoot_forces"] = flfoot_forces; cfg["output"]["plan_time_4"]["frfoot_forces"] = frfoot_forces;
    cfg["output"]["plan_time_4"]["rlfoot_forces"] = rlfoot_forces; cfg["output"]["plan_time_4"]["rrfoot_forces"] = rrfoot_forces;
    // Timestep 5
    flfoot_forces[0] = sp_->mpc_rxn_forces[48+0]; flfoot_forces[1] = sp_->mpc_rxn_forces[48+1]; flfoot_forces[2] = sp_->mpc_rxn_forces[48+2];
    frfoot_forces[0] = sp_->mpc_rxn_forces[48+3]; frfoot_forces[1] = sp_->mpc_rxn_forces[48+4]; frfoot_forces[2] = sp_->mpc_rxn_forces[48+5];
    rlfoot_forces[0] = sp_->mpc_rxn_forces[48+6]; rlfoot_forces[1] = sp_->mpc_rxn_forces[48+7]; rlfoot_forces[2] = sp_->mpc_rxn_forces[48+8];
    rrfoot_forces[0] = sp_->mpc_rxn_forces[48+9]; rrfoot_forces[1] = sp_->mpc_rxn_forces[48+10];rrfoot_forces[2] = sp_->mpc_rxn_forces[48+11];
    cfg["output"]["plan_time_5"]["flfoot_forces"] = flfoot_forces; cfg["output"]["plan_time_5"]["frfoot_forces"] = frfoot_forces;
    cfg["output"]["plan_time_5"]["rlfoot_forces"] = rlfoot_forces; cfg["output"]["plan_time_5"]["rrfoot_forces"] = rrfoot_forces;
    // Timestep 6
    flfoot_forces[0] = sp_->mpc_rxn_forces[60+0]; flfoot_forces[1] = sp_->mpc_rxn_forces[60+1]; flfoot_forces[2] = sp_->mpc_rxn_forces[60+2];
    frfoot_forces[0] = sp_->mpc_rxn_forces[60+3]; frfoot_forces[1] = sp_->mpc_rxn_forces[60+4]; frfoot_forces[2] = sp_->mpc_rxn_forces[60+5];
    rlfoot_forces[0] = sp_->mpc_rxn_forces[60+6]; rlfoot_forces[1] = sp_->mpc_rxn_forces[60+7]; rlfoot_forces[2] = sp_->mpc_rxn_forces[60+8];
    rrfoot_forces[0] = sp_->mpc_rxn_forces[60+9]; rrfoot_forces[1] = sp_->mpc_rxn_forces[60+10];rrfoot_forces[2] = sp_->mpc_rxn_forces[60+11];
    cfg["output"]["plan_time_6"]["flfoot_forces"] = flfoot_forces; cfg["output"]["plan_time_6"]["frfoot_forces"] = frfoot_forces;
    cfg["output"]["plan_time_6"]["rlfoot_forces"] = rlfoot_forces; cfg["output"]["plan_time_6"]["rrfoot_forces"] = rrfoot_forces;
    /*// Timestep 7
    flfoot_forces[0] = sp_->mpc_rxn_forces[72+0]; flfoot_forces[1] = sp_->mpc_rxn_forces[72+1]; flfoot_forces[2] = sp_->mpc_rxn_forces[72+2];
    frfoot_forces[0] = sp_->mpc_rxn_forces[72+3]; frfoot_forces[1] = sp_->mpc_rxn_forces[72+4]; frfoot_forces[2] = sp_->mpc_rxn_forces[72+5];
    rlfoot_forces[0] = sp_->mpc_rxn_forces[72+6]; rlfoot_forces[1] = sp_->mpc_rxn_forces[72+7]; rlfoot_forces[2] = sp_->mpc_rxn_forces[72+8];
    rrfoot_forces[0] = sp_->mpc_rxn_forces[72+9]; rrfoot_forces[1] = sp_->mpc_rxn_forces[72+10];rrfoot_forces[2] = sp_->mpc_rxn_forces[72+11];
    cfg["output"]["plan_time_7"]["flfoot_forces"] = flfoot_forces; cfg["output"]["plan_time_7"]["frfoot_forces"] = frfoot_forces;
    cfg["output"]["plan_time_7"]["rlfoot_forces"] = rlfoot_forces; cfg["output"]["plan_time_7"]["rrfoot_forces"] = rrfoot_forces;
    // Timestep 8
    flfoot_forces[0] = sp_->mpc_rxn_forces[84+0]; flfoot_forces[1] = sp_->mpc_rxn_forces[84+1]; flfoot_forces[2] = sp_->mpc_rxn_forces[84+2];
    frfoot_forces[0] = sp_->mpc_rxn_forces[84+3]; frfoot_forces[1] = sp_->mpc_rxn_forces[84+4]; frfoot_forces[2] = sp_->mpc_rxn_forces[84+5];
    rlfoot_forces[0] = sp_->mpc_rxn_forces[84+6]; rlfoot_forces[1] = sp_->mpc_rxn_forces[84+7]; rlfoot_forces[2] = sp_->mpc_rxn_forces[84+8];
    rrfoot_forces[0] = sp_->mpc_rxn_forces[84+9]; rrfoot_forces[1] = sp_->mpc_rxn_forces[84+10];rrfoot_forces[2] = sp_->mpc_rxn_forces[84+11];
    cfg["output"]["plan_time_8"]["flfoot_forces"] = flfoot_forces; cfg["output"]["plan_time_8"]["frfoot_forces"] = frfoot_forces;
    cfg["output"]["plan_time_8"]["rlfoot_forces"] = rlfoot_forces; cfg["output"]["plan_time_8"]["rrfoot_forces"] = rrfoot_forces;
    // Timestep 9
    flfoot_forces[0] = sp_->mpc_rxn_forces[96+0]; flfoot_forces[1] = sp_->mpc_rxn_forces[96+1]; flfoot_forces[2] = sp_->mpc_rxn_forces[96+2];
    frfoot_forces[0] = sp_->mpc_rxn_forces[96+3]; frfoot_forces[1] = sp_->mpc_rxn_forces[96+4]; frfoot_forces[2] = sp_->mpc_rxn_forces[96+5];
    rlfoot_forces[0] = sp_->mpc_rxn_forces[96+6]; rlfoot_forces[1] = sp_->mpc_rxn_forces[96+7]; rlfoot_forces[2] = sp_->mpc_rxn_forces[96+8];
    rrfoot_forces[0] = sp_->mpc_rxn_forces[96+9]; rrfoot_forces[1] = sp_->mpc_rxn_forces[96+10];rrfoot_forces[2] = sp_->mpc_rxn_forces[96+11];
    cfg["output"]["plan_time_9"]["flfoot_forces"] = flfoot_forces; cfg["output"]["plan_time_9"]["frfoot_forces"] = frfoot_forces;
    cfg["output"]["plan_time_9"]["rlfoot_forces"] = rlfoot_forces; cfg["output"]["plan_time_9"]["rrfoot_forces"] = rrfoot_forces;
    // Timestep 10
    flfoot_forces[0] = sp_->mpc_rxn_forces[108+0]; flfoot_forces[1] = sp_->mpc_rxn_forces[108+1]; flfoot_forces[2] = sp_->mpc_rxn_forces[108+2];
    frfoot_forces[0] = sp_->mpc_rxn_forces[108+3]; frfoot_forces[1] = sp_->mpc_rxn_forces[108+4]; frfoot_forces[2] = sp_->mpc_rxn_forces[108+5];
    rlfoot_forces[0] = sp_->mpc_rxn_forces[108+6]; rlfoot_forces[1] = sp_->mpc_rxn_forces[108+7]; rlfoot_forces[2] = sp_->mpc_rxn_forces[108+8];
    rrfoot_forces[0] = sp_->mpc_rxn_forces[108+9]; rrfoot_forces[1] = sp_->mpc_rxn_forces[108+10];rrfoot_forces[2] = sp_->mpc_rxn_forces[108+11];
    cfg["output"]["plan_time_10"]["flfoot_forces"] = flfoot_forces; cfg["output"]["plan_time_10"]["frfoot_forces"] = frfoot_forces;
    cfg["output"]["plan_time_10"]["rlfoot_forces"] = rlfoot_forces; cfg["output"]["plan_time_10"]["rrfoot_forces"] = rrfoot_forces;*/

    // =====================================================================
    // MPC Internal State Progression
    // =====================================================================
    // Timestep 1
    cfg["internal_state"]["plan_time_1"]["roll"] = state_progression_[0];
    cfg["internal_state"]["plan_time_1"]["pitch"] = state_progression_[1];
    cfg["internal_state"]["plan_time_1"]["yaw"] = state_progression_[2];
    cfg["internal_state"]["plan_time_1"]["com_x"] = state_progression_[3];
    cfg["internal_state"]["plan_time_1"]["com_y"] = state_progression_[4];
    cfg["internal_state"]["plan_time_1"]["com_z"] = state_progression_[5];
    cfg["internal_state"]["plan_time_1"]["com_ang_vel0"] = state_progression_[6];
    cfg["internal_state"]["plan_time_1"]["com_ang_vel1"] = state_progression_[7];
    cfg["internal_state"]["plan_time_1"]["com_ang_vel2"] = state_progression_[8];
    cfg["internal_state"]["plan_time_1"]["com_x_vel"] = state_progression_[9];
    cfg["internal_state"]["plan_time_1"]["com_y_vel"] = state_progression_[10];
    cfg["internal_state"]["plan_time_1"]["com_z_vel"] = state_progression_[11];

    cfg["internal_state"]["plan_time_1"]["roll_des"] = des_states_[0];
    cfg["internal_state"]["plan_time_1"]["pitch_des"] = des_states_[1];
    cfg["internal_state"]["plan_time_1"]["yaw_des"] = des_states_[2];
    cfg["internal_state"]["plan_time_1"]["com_x_des"] = des_states_[3];
    cfg["internal_state"]["plan_time_1"]["com_y_des"] = des_states_[4];
    cfg["internal_state"]["plan_time_1"]["com_z_des"] = des_states_[5];
    cfg["internal_state"]["plan_time_1"]["com_ang_vel0_des"] = des_states_[6];
    cfg["internal_state"]["plan_time_1"]["com_ang_vel1_des"] = des_states_[7];
    cfg["internal_state"]["plan_time_1"]["com_ang_vel2_des"] = des_states_[8];
    cfg["internal_state"]["plan_time_1"]["com_x_vel_des"] = des_states_[9];
    cfg["internal_state"]["plan_time_1"]["com_y_vel_des"] = des_states_[10];
    cfg["internal_state"]["plan_time_1"]["com_z_vel_des"] = des_states_[11];
    // Timestep 2
    cfg["internal_state"]["plan_time_2"]["roll"] = state_progression_[13];
    cfg["internal_state"]["plan_time_2"]["pitch"] = state_progression_[14];
    cfg["internal_state"]["plan_time_2"]["yaw"] = state_progression_[15];
    cfg["internal_state"]["plan_time_2"]["com_x"] = state_progression_[16];
    cfg["internal_state"]["plan_time_2"]["com_y"] = state_progression_[17];
    cfg["internal_state"]["plan_time_2"]["com_z"] = state_progression_[18];
    cfg["internal_state"]["plan_time_2"]["com_ang_vel0"] = state_progression_[19];
    cfg["internal_state"]["plan_time_2"]["com_ang_vel1"] = state_progression_[20];
    cfg["internal_state"]["plan_time_2"]["com_ang_vel2"] = state_progression_[21];
    cfg["internal_state"]["plan_time_2"]["com_x_vel"] = state_progression_[22];
    cfg["internal_state"]["plan_time_2"]["com_y_vel"] = state_progression_[23];
    cfg["internal_state"]["plan_time_2"]["com_z_vel"] = state_progression_[24];

    cfg["internal_state"]["plan_time_2"]["roll_des"] = des_states_[13];
    cfg["internal_state"]["plan_time_2"]["pitch_des"] = des_states_[14];
    cfg["internal_state"]["plan_time_2"]["yaw_des"] = des_states_[15];
    cfg["internal_state"]["plan_time_2"]["com_x_des"] = des_states_[16];
    cfg["internal_state"]["plan_time_2"]["com_y_des"] = des_states_[17];
    cfg["internal_state"]["plan_time_2"]["com_z_des"] = des_states_[18];
    cfg["internal_state"]["plan_time_2"]["com_ang_vel0_des"] = des_states_[19];
    cfg["internal_state"]["plan_time_2"]["com_ang_vel1_des"] = des_states_[20];
    cfg["internal_state"]["plan_time_2"]["com_ang_vel2_des"] = des_states_[21];
    cfg["internal_state"]["plan_time_2"]["com_x_vel_des"] = des_states_[22];
    cfg["internal_state"]["plan_time_2"]["com_y_vel_des"] = des_states_[23];
    cfg["internal_state"]["plan_time_2"]["com_z_vel_des"] = des_states_[24];
    // Timestep 3
    cfg["internal_state"]["plan_time_3"]["roll"] = state_progression_[0 + 26];
    cfg["internal_state"]["plan_time_3"]["pitch"] = state_progression_[1 + 26];
    cfg["internal_state"]["plan_time_3"]["yaw"] = state_progression_[2 + 26];
    cfg["internal_state"]["plan_time_3"]["com_x"] = state_progression_[3 + 26];
    cfg["internal_state"]["plan_time_3"]["com_y"] = state_progression_[4 + 26];
    cfg["internal_state"]["plan_time_3"]["com_z"] = state_progression_[5 + 26];
    cfg["internal_state"]["plan_time_3"]["com_ang_vel0"] = state_progression_[6 + 26];
    cfg["internal_state"]["plan_time_3"]["com_ang_vel1"] = state_progression_[7 + 26];
    cfg["internal_state"]["plan_time_3"]["com_ang_vel2"] = state_progression_[8 + 26];
    cfg["internal_state"]["plan_time_3"]["com_x_vel"] = state_progression_[9 + 26];
    cfg["internal_state"]["plan_time_3"]["com_y_vel"] = state_progression_[10 + 26];
    cfg["internal_state"]["plan_time_3"]["com_z_vel"] = state_progression_[11 + 26];

    cfg["internal_state"]["plan_time_3"]["roll_des"] = des_states_[0 + 26];
    cfg["internal_state"]["plan_time_3"]["pitch_des"] = des_states_[1 + 26];
    cfg["internal_state"]["plan_time_3"]["yaw_des"] = des_states_[2 + 26];
    cfg["internal_state"]["plan_time_3"]["com_x_des"] = des_states_[3 + 26];
    cfg["internal_state"]["plan_time_3"]["com_y_des"] = des_states_[4 + 26];
    cfg["internal_state"]["plan_time_3"]["com_z_des"] = des_states_[5 + 26];
    cfg["internal_state"]["plan_time_3"]["com_ang_vel0_des"] = des_states_[6 + 26];
    cfg["internal_state"]["plan_time_3"]["com_ang_vel1_des"] = des_states_[7 + 26];
    cfg["internal_state"]["plan_time_3"]["com_ang_vel2_des"] = des_states_[8 + 26];
    cfg["internal_state"]["plan_time_3"]["com_x_vel_des"] = des_states_[9 + 26];
    cfg["internal_state"]["plan_time_3"]["com_y_vel_des"] = des_states_[10 + 26];
    cfg["internal_state"]["plan_time_3"]["com_z_vel_des"] = des_states_[11 + 26];
    // Timestep 4
    cfg["internal_state"]["plan_time_4"]["roll"] = state_progression_[0 + 39];
    cfg["internal_state"]["plan_time_4"]["pitch"] = state_progression_[1 + 39];
    cfg["internal_state"]["plan_time_4"]["yaw"] = state_progression_[2 + 39];
    cfg["internal_state"]["plan_time_4"]["com_x"] = state_progression_[3 + 39];
    cfg["internal_state"]["plan_time_4"]["com_y"] = state_progression_[4 + 39];
    cfg["internal_state"]["plan_time_4"]["com_z"] = state_progression_[5 + 39];
    cfg["internal_state"]["plan_time_4"]["com_ang_vel0"] = state_progression_[6 + 39];
    cfg["internal_state"]["plan_time_4"]["com_ang_vel1"] = state_progression_[7 + 39];
    cfg["internal_state"]["plan_time_4"]["com_ang_vel2"] = state_progression_[8 + 39];
    cfg["internal_state"]["plan_time_4"]["com_x_vel"] = state_progression_[9 + 39];
    cfg["internal_state"]["plan_time_4"]["com_y_vel"] = state_progression_[10 + 39];
    cfg["internal_state"]["plan_time_4"]["com_z_vel"] = state_progression_[11 + 39];

    cfg["internal_state"]["plan_time_4"]["roll_des"] = des_states_[0 + 39];
    cfg["internal_state"]["plan_time_4"]["pitch_des"] = des_states_[1 + 39];
    cfg["internal_state"]["plan_time_4"]["yaw_des"] = des_states_[2 + 39];
    cfg["internal_state"]["plan_time_4"]["com_x_des"] = des_states_[3 + 39];
    cfg["internal_state"]["plan_time_4"]["com_y_des"] = des_states_[4 + 39];
    cfg["internal_state"]["plan_time_4"]["com_z_des"] = des_states_[5 + 39];
    cfg["internal_state"]["plan_time_4"]["com_ang_vel0_des"] = des_states_[6 + 39];
    cfg["internal_state"]["plan_time_4"]["com_ang_vel1_des"] = des_states_[7 + 39];
    cfg["internal_state"]["plan_time_4"]["com_ang_vel2_des"] = des_states_[8 + 39];
    cfg["internal_state"]["plan_time_4"]["com_x_vel_des"] = des_states_[9 + 39];
    cfg["internal_state"]["plan_time_4"]["com_y_vel_des"] = des_states_[10 + 39];
    cfg["internal_state"]["plan_time_4"]["com_z_vel_des"] = des_states_[11 + 39];
    // Timestep 5
    cfg["internal_state"]["plan_time_5"]["roll"] = state_progression_[0 + 52];
    cfg["internal_state"]["plan_time_5"]["pitch"] = state_progression_[1 + 52];
    cfg["internal_state"]["plan_time_5"]["yaw"] = state_progression_[2 + 52];
    cfg["internal_state"]["plan_time_5"]["com_x"] = state_progression_[3 + 52];
    cfg["internal_state"]["plan_time_5"]["com_y"] = state_progression_[4 + 52];
    cfg["internal_state"]["plan_time_5"]["com_z"] = state_progression_[5 + 52];
    cfg["internal_state"]["plan_time_5"]["com_ang_vel0"] = state_progression_[6 + 52];
    cfg["internal_state"]["plan_time_5"]["com_ang_vel1"] = state_progression_[7 + 52];
    cfg["internal_state"]["plan_time_5"]["com_ang_vel2"] = state_progression_[8 + 52];
    cfg["internal_state"]["plan_time_5"]["com_x_vel"] = state_progression_[9 + 52];
    cfg["internal_state"]["plan_time_5"]["com_y_vel"] = state_progression_[10 + 52];
    cfg["internal_state"]["plan_time_5"]["com_z_vel"] = state_progression_[11 + 52];

    cfg["internal_state"]["plan_time_5"]["roll_des"] = des_states_[0 + 52];
    cfg["internal_state"]["plan_time_5"]["pitch_des"] = des_states_[1 + 52];
    cfg["internal_state"]["plan_time_5"]["yaw_des"] = des_states_[2 + 52];
    cfg["internal_state"]["plan_time_5"]["com_x_des"] = des_states_[3 + 52];
    cfg["internal_state"]["plan_time_5"]["com_y_des"] = des_states_[4 + 52];
    cfg["internal_state"]["plan_time_5"]["com_z_des"] = des_states_[5 + 52];
    cfg["internal_state"]["plan_time_5"]["com_ang_vel0_des"] = des_states_[6 + 52];
    cfg["internal_state"]["plan_time_5"]["com_ang_vel1_des"] = des_states_[7 + 52];
    cfg["internal_state"]["plan_time_5"]["com_ang_vel2_des"] = des_states_[8 + 52];
    cfg["internal_state"]["plan_time_5"]["com_x_vel_des"] = des_states_[9 + 52];
    cfg["internal_state"]["plan_time_5"]["com_y_vel_des"] = des_states_[10 + 52];
    cfg["internal_state"]["plan_time_5"]["com_z_vel_des"] = des_states_[11 + 52];
    // Timestep 6
    cfg["internal_state"]["plan_time_6"]["roll"] = state_progression_[0 + 65];
    cfg["internal_state"]["plan_time_6"]["pitch"] = state_progression_[1 + 65];
    cfg["internal_state"]["plan_time_6"]["yaw"] = state_progression_[2 + 65];
    cfg["internal_state"]["plan_time_6"]["com_x"] = state_progression_[3 + 65];
    cfg["internal_state"]["plan_time_6"]["com_y"] = state_progression_[4 + 65];
    cfg["internal_state"]["plan_time_6"]["com_z"] = state_progression_[5 + 65];
    cfg["internal_state"]["plan_time_6"]["com_ang_vel0"] = state_progression_[6 + 65];
    cfg["internal_state"]["plan_time_6"]["com_ang_vel1"] = state_progression_[7 + 65];
    cfg["internal_state"]["plan_time_6"]["com_ang_vel2"] = state_progression_[8 + 65];
    cfg["internal_state"]["plan_time_6"]["com_x_vel"] = state_progression_[9 + 65];
    cfg["internal_state"]["plan_time_6"]["com_y_vel"] = state_progression_[10 + 65];
    cfg["internal_state"]["plan_time_6"]["com_z_vel"] = state_progression_[11 + 65];

    cfg["internal_state"]["plan_time_6"]["roll_des"] = des_states_[0 + 65];
    cfg["internal_state"]["plan_time_6"]["pitch_des"] = des_states_[1 + 65];
    cfg["internal_state"]["plan_time_6"]["yaw_des"] = des_states_[2 + 65];
    cfg["internal_state"]["plan_time_6"]["com_x_des"] = des_states_[3 + 65];
    cfg["internal_state"]["plan_time_6"]["com_y_des"] = des_states_[4 + 65];
    cfg["internal_state"]["plan_time_6"]["com_z_des"] = des_states_[5 + 65];
    cfg["internal_state"]["plan_time_6"]["com_ang_vel0_des"] = des_states_[6 + 65];
    cfg["internal_state"]["plan_time_6"]["com_ang_vel1_des"] = des_states_[7 + 65];
    cfg["internal_state"]["plan_time_6"]["com_ang_vel2_des"] = des_states_[8 + 65];
    cfg["internal_state"]["plan_time_6"]["com_x_vel_des"] = des_states_[9 + 65];
    cfg["internal_state"]["plan_time_6"]["com_y_vel_des"] = des_states_[10 + 65];
    cfg["internal_state"]["plan_time_6"]["com_z_vel_des"] = des_states_[11 + 65];
    /*// Timestep 7
    cfg["internal_state"]["plan_time_7"]["roll"] = state_progression_[0 + 78];
    cfg["internal_state"]["plan_time_7"]["pitch"] = state_progression_[1 + 78];
    cfg["internal_state"]["plan_time_7"]["yaw"] = state_progression_[2 + 78];
    cfg["internal_state"]["plan_time_7"]["com_x"] = state_progression_[3 + 78];
    cfg["internal_state"]["plan_time_7"]["com_y"] = state_progression_[4 + 78];
    cfg["internal_state"]["plan_time_7"]["com_z"] = state_progression_[5 + 78];
    cfg["internal_state"]["plan_time_7"]["com_ang_vel0"] = state_progression_[6 + 78];
    cfg["internal_state"]["plan_time_7"]["com_ang_vel1"] = state_progression_[7 + 78];
    cfg["internal_state"]["plan_time_7"]["com_ang_vel2"] = state_progression_[8 + 78];
    cfg["internal_state"]["plan_time_7"]["com_x_vel"] = state_progression_[9 + 78];
    cfg["internal_state"]["plan_time_7"]["com_y_vel"] = state_progression_[10 + 78];
    cfg["internal_state"]["plan_time_7"]["com_z_vel"] = state_progression_[11 + 78];

    cfg["internal_state"]["plan_time_7"]["roll_des"] = des_states_[0 + 78];
    cfg["internal_state"]["plan_time_7"]["pitch_des"] = des_states_[1 + 78];
    cfg["internal_state"]["plan_time_7"]["yaw_des"] = des_states_[2 + 78];
    cfg["internal_state"]["plan_time_7"]["com_x_des"] = des_states_[3 + 78];
    cfg["internal_state"]["plan_time_7"]["com_y_des"] = des_states_[4 + 78];
    cfg["internal_state"]["plan_time_7"]["com_z_des"] = des_states_[5 + 78];
    cfg["internal_state"]["plan_time_7"]["com_ang_vel0_des"] = des_states_[6 + 78];
    cfg["internal_state"]["plan_time_7"]["com_ang_vel1_des"] = des_states_[7 + 78];
    cfg["internal_state"]["plan_time_7"]["com_ang_vel2_des"] = des_states_[8 + 78];
    cfg["internal_state"]["plan_time_7"]["com_x_vel_des"] = des_states_[9 + 78];
    cfg["internal_state"]["plan_time_7"]["com_y_vel_des"] = des_states_[10 + 78];
    cfg["internal_state"]["plan_time_7"]["com_z_vel_des"] = des_states_[11 + 78];
    // Timestep 8
    cfg["internal_state"]["plan_time_8"]["roll"] = state_progression_[0 + 91];
    cfg["internal_state"]["plan_time_8"]["pitch"] = state_progression_[1 + 91];
    cfg["internal_state"]["plan_time_8"]["yaw"] = state_progression_[2 + 91];
    cfg["internal_state"]["plan_time_8"]["com_x"] = state_progression_[3 + 91];
    cfg["internal_state"]["plan_time_8"]["com_y"] = state_progression_[4 + 91];
    cfg["internal_state"]["plan_time_8"]["com_z"] = state_progression_[5 + 91];
    cfg["internal_state"]["plan_time_8"]["com_ang_vel0"] = state_progression_[6 + 91];
    cfg["internal_state"]["plan_time_8"]["com_ang_vel1"] = state_progression_[7 + 91];
    cfg["internal_state"]["plan_time_8"]["com_ang_vel2"] = state_progression_[8 + 91];
    cfg["internal_state"]["plan_time_8"]["com_x_vel"] = state_progression_[9 + 91];
    cfg["internal_state"]["plan_time_8"]["com_y_vel"] = state_progression_[10 + 91];
    cfg["internal_state"]["plan_time_8"]["com_z_vel"] = state_progression_[11 + 91];

    cfg["internal_state"]["plan_time_8"]["roll_des"] = des_states_[0 + 91];
    cfg["internal_state"]["plan_time_8"]["pitch_des"] = des_states_[1 + 91];
    cfg["internal_state"]["plan_time_8"]["yaw_des"] = des_states_[2 + 91];
    cfg["internal_state"]["plan_time_8"]["com_x_des"] = des_states_[3 + 91];
    cfg["internal_state"]["plan_time_8"]["com_y_des"] = des_states_[4 + 91];
    cfg["internal_state"]["plan_time_8"]["com_z_des"] = des_states_[5 + 91];
    cfg["internal_state"]["plan_time_8"]["com_ang_vel0_des"] = des_states_[6 + 91];
    cfg["internal_state"]["plan_time_8"]["com_ang_vel1_des"] = des_states_[7 + 91];
    cfg["internal_state"]["plan_time_8"]["com_ang_vel2_des"] = des_states_[8 + 91];
    cfg["internal_state"]["plan_time_8"]["com_x_vel_des"] = des_states_[9 + 91];
    cfg["internal_state"]["plan_time_8"]["com_y_vel_des"] = des_states_[10 + 91];
    cfg["internal_state"]["plan_time_8"]["com_z_vel_des"] = des_states_[11 + 91];
    // Timestep 9
    cfg["internal_state"]["plan_time_9"]["roll"] = state_progression_[0 + 104];
    cfg["internal_state"]["plan_time_9"]["pitch"] = state_progression_[1 + 104];
    cfg["internal_state"]["plan_time_9"]["yaw"] = state_progression_[2 + 104];
    cfg["internal_state"]["plan_time_9"]["com_x"] = state_progression_[3 + 104];
    cfg["internal_state"]["plan_time_9"]["com_y"] = state_progression_[4 + 104];
    cfg["internal_state"]["plan_time_9"]["com_z"] = state_progression_[5 + 104];
    cfg["internal_state"]["plan_time_9"]["com_ang_vel0"] = state_progression_[6 + 104];
    cfg["internal_state"]["plan_time_9"]["com_ang_vel1"] = state_progression_[7 + 104];
    cfg["internal_state"]["plan_time_9"]["com_ang_vel2"] = state_progression_[8 + 104];
    cfg["internal_state"]["plan_time_9"]["com_x_vel"] = state_progression_[9 + 104];
    cfg["internal_state"]["plan_time_9"]["com_y_vel"] = state_progression_[10 + 104];
    cfg["internal_state"]["plan_time_9"]["com_z_vel"] = state_progression_[11 + 104];

    cfg["internal_state"]["plan_time_9"]["roll_des"] = des_states_[0 + 104];
    cfg["internal_state"]["plan_time_9"]["pitch_des"] = des_states_[1 + 104];
    cfg["internal_state"]["plan_time_9"]["yaw_des"] = des_states_[2 + 104];
    cfg["internal_state"]["plan_time_9"]["com_x_des"] = des_states_[3 + 104];
    cfg["internal_state"]["plan_time_9"]["com_y_des"] = des_states_[4 + 104];
    cfg["internal_state"]["plan_time_9"]["com_z_des"] = des_states_[5 + 104];
    cfg["internal_state"]["plan_time_9"]["com_ang_vel0_des"] = des_states_[6 + 104];
    cfg["internal_state"]["plan_time_9"]["com_ang_vel1_des"] = des_states_[7 + 104];
    cfg["internal_state"]["plan_time_9"]["com_ang_vel2_des"] = des_states_[8 + 104];
    cfg["internal_state"]["plan_time_9"]["com_x_vel_des"] = des_states_[9 + 104];
    cfg["internal_state"]["plan_time_9"]["com_y_vel_des"] = des_states_[10 + 104];
    cfg["internal_state"]["plan_time_9"]["com_z_vel_des"] = des_states_[11 + 104];
    // Timestep 10
    cfg["internal_state"]["plan_time_10"]["roll"] = state_progression_[0 + 117];
    cfg["internal_state"]["plan_time_10"]["pitch"] = state_progression_[1 + 117];
    cfg["internal_state"]["plan_time_10"]["yaw"] = state_progression_[2 + 117];
    cfg["internal_state"]["plan_time_10"]["com_x"] = state_progression_[3 + 117];
    cfg["internal_state"]["plan_time_10"]["com_y"] = state_progression_[4 + 117];
    cfg["internal_state"]["plan_time_10"]["com_z"] = state_progression_[5 + 117];
    cfg["internal_state"]["plan_time_10"]["com_ang_vel0"] = state_progression_[6 + 117];
    cfg["internal_state"]["plan_time_10"]["com_ang_vel1"] = state_progression_[7 + 117];
    cfg["internal_state"]["plan_time_10"]["com_ang_vel2"] = state_progression_[8 + 117];
    cfg["internal_state"]["plan_time_10"]["com_x_vel"] = state_progression_[9 + 117];
    cfg["internal_state"]["plan_time_10"]["com_y_vel"] = state_progression_[10 + 117];
    cfg["internal_state"]["plan_time_10"]["com_z_vel"] = state_progression_[11 + 117];

    cfg["internal_state"]["plan_time_10"]["roll_des"] = des_states_[0 + 117];
    cfg["internal_state"]["plan_time_10"]["pitch_des"] = des_states_[1 + 117];
    cfg["internal_state"]["plan_time_10"]["yaw_des"] = des_states_[2 + 117];
    cfg["internal_state"]["plan_time_10"]["com_x_des"] = des_states_[3 + 117];
    cfg["internal_state"]["plan_time_10"]["com_y_des"] = des_states_[4 + 117];
    cfg["internal_state"]["plan_time_10"]["com_z_des"] = des_states_[5 + 117];
    cfg["internal_state"]["plan_time_10"]["com_ang_vel0_des"] = des_states_[6 + 117];
    cfg["internal_state"]["plan_time_10"]["com_ang_vel1_des"] = des_states_[7 + 117];
    cfg["internal_state"]["plan_time_10"]["com_ang_vel2_des"] = des_states_[8 + 117];
    cfg["internal_state"]["plan_time_10"]["com_x_vel_des"] = des_states_[9 + 117];
    cfg["internal_state"]["plan_time_10"]["com_y_vel_des"] = des_states_[10 + 117];
    cfg["internal_state"]["plan_time_10"]["com_z_vel_des"] = des_states_[11 + 117];*/
 

    int plan_step = t_start;
    std::string full_path = THIS_COM + std::string("ExperimentData/mpc_io/mpc_io_") +
                            std::to_string(num_mpc_calls) + std::string(".yaml");
    std::ofstream file_out(full_path);
    file_out << cfg;
  } catch (YAML::ParserException& e) {
      std::cout << e.what() << std::endl;
  }


}

void A1ControlArchitecture::getCommand(void* _command) {
  // Initialize State
  if (b_state_first_visit_) {
    state_machines_[state_]->firstVisit();
    b_state_first_visit_ = false;
  }
  //if(state_ != A1_STATES::BALANCE && state_ != A1_STATES::STAND){
    if(mpc_counter >= 6 || state_holder_for_mpc_ != state_){// Call the MPC at 83.3 Hz
        // Start measuring time
        // struct timeval begin, end;
        // gettimeofday(&begin, 0);
        solveMPC();
        // gettimeofday(&end, 0);
        // long seconds = end.tv_sec - begin.tv_sec;
        // long microseconds = end.tv_usec - begin.tv_usec;
        // double elapsed = seconds + microseconds*1e-6;
        // printf("Time measured: %.3f seconds.\n", elapsed);
        if(sp_->mpc_rxn_forces.size() > 10){
          rxn_force_manager_->updateSolution(
                        sp_->curr_time,
                        sp_->mpc_rxn_forces);
          mpc_counter = 0;
          state_holder_for_mpc_ = state_;
        }
    } else {++mpc_counter;}
    // Get the interpolated value for reaction forces from previous MPC call
    // command_rxn_forces = rxn_force_manager_->getRFSolution(sp_->curr_time);
    // myUtils::pretty_print(command_rxn_forces,std::cout, "interpolated RXN forces");
    for(int i=0; i<12; ++i){
        command_rxn_forces[i] = sp_->mpc_rxn_forces[i];
    }

    // Set the Contact Level Rxn Forces
    Eigen::VectorXd tmp_rxn_forces; tmp_rxn_forces = Eigen::VectorXd::Zero(3);
    tmp_rxn_forces[0] = -command_rxn_forces[0];
    tmp_rxn_forces[1] = -command_rxn_forces[1];// y frame in MPC is our -y
    tmp_rxn_forces[2] = -command_rxn_forces[2];// z frame in MPC is our -z
    command_rxn_forces[0] = -command_rxn_forces[0];
    command_rxn_forces[1] = -command_rxn_forces[1];
    command_rxn_forces[2] = -command_rxn_forces[2];
    taf_container_->flfoot_contact_->setRFDesired(tmp_rxn_forces);
    tmp_rxn_forces[0] = -command_rxn_forces[3];
    tmp_rxn_forces[1] = -command_rxn_forces[4];
    tmp_rxn_forces[2] = -command_rxn_forces[5];
    command_rxn_forces[3] = -command_rxn_forces[3];
    command_rxn_forces[4] = -command_rxn_forces[4];
    command_rxn_forces[5] = -command_rxn_forces[5];
    taf_container_->frfoot_contact_->setRFDesired(tmp_rxn_forces);
    tmp_rxn_forces[0] = -command_rxn_forces[6];
    tmp_rxn_forces[1] = -command_rxn_forces[7];
    tmp_rxn_forces[2] = -command_rxn_forces[8];
    command_rxn_forces[6] = -command_rxn_forces[6];
    command_rxn_forces[7] = -command_rxn_forces[7];
    command_rxn_forces[8] = -command_rxn_forces[8];
    taf_container_->rlfoot_contact_->setRFDesired(tmp_rxn_forces);
    tmp_rxn_forces[0] = -command_rxn_forces[9];
    tmp_rxn_forces[1] = -command_rxn_forces[10];
    tmp_rxn_forces[2] = -command_rxn_forces[11];
    command_rxn_forces[9] = -command_rxn_forces[9];
    command_rxn_forces[10] = -command_rxn_forces[10];
    command_rxn_forces[11] = -command_rxn_forces[11];
    taf_container_->rrfoot_contact_->setRFDesired(tmp_rxn_forces);
  // }
  // if(state_ == A1_STATES::FL_SWING || state_ == A1_STATES::FR_SWING) {
  //   state_machines_[state_]->updateTestCounter(test_counter);
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
  Eigen::VectorXd Fr_result_, Fr_correct_length_result_;
  Fr_correct_length_result_ = Eigen::VectorXd::Zero(12);
  Fr_result_ = main_controller_->getCommand(_command, change_qp_weights_for_walking);
  if(Fr_result_.size() < 12) {
    // std::cout << "Fr_result_.size() = " << Fr_result_.size() << std::endl;
    if(state_ == A1_STATES::FL_HALF_SWING || state_ == A1_STATES::FL_FINAL_SWING) {
        Fr_correct_length_result_[3] = Fr_result_[0];
        Fr_correct_length_result_[4] = Fr_result_[1];
        Fr_correct_length_result_[5] = Fr_result_[2];
        Fr_correct_length_result_[6] = Fr_result_[3];
        Fr_correct_length_result_[7] = Fr_result_[4];
        Fr_correct_length_result_[8] = Fr_result_[5];
    }
    if(state_ == A1_STATES::FR_HALF_SWING || state_ == A1_STATES::FR_FINAL_SWING) {
        Fr_correct_length_result_[0] = Fr_result_[0];
        Fr_correct_length_result_[1] = Fr_result_[1];
        Fr_correct_length_result_[2] = Fr_result_[2];
        Fr_correct_length_result_[9] = Fr_result_[3];
        Fr_correct_length_result_[10] = Fr_result_[4];
        Fr_correct_length_result_[11] = Fr_result_[5];
    }
    sp_->final_reaction_forces = Fr_correct_length_result_;
  } else {
    sp_->final_reaction_forces = Fr_result_;
  }
  // Save Data
  saveData();

  ++test_counter;

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
  state_machines_[A1_STATES::FR_HALF_SWING]->initialization(cfg_["state_swing"]);
  state_machines_[A1_STATES::FL_HALF_SWING]->initialization(cfg_["state_swing"]);
  state_machines_[A1_STATES::FR_FINAL_SWING]->initialization(cfg_["state_swing"]);
  state_machines_[A1_STATES::FL_FINAL_SWING]->initialization(cfg_["state_swing"]);
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
