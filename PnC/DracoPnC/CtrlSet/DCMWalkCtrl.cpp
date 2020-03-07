#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/WBC/IHWBC/IHWBC.hpp>

#include <Utils/IO/DataManager.hpp>
#include <Utils/Math/MathUtilities.hpp>
#include <PnC/MPC/MPCDesiredTrajectoryManager.hpp>

#include <PnC/DracoPnC/PredictionModule/DracoFootstep.hpp>
#include <PnC/DracoPnC/PredictionModule/WalkingReferenceTrajectoryModule.hpp>
#include <PnC/DracoPnC/PredictionModule/DCMWalkingReferenceTrajectoryModule.hpp>

// interpolators
#include <Utils/Math/hermite_curve_vec.hpp>
#include <Utils/Math/hermite_quaternion_curve.hpp>

DCMWalkCtrl::DCMWalkCtrl(RobotSystem* robot) : Controller(robot) {
    myUtils::pretty_constructor(2, "DCM Walk Ctrl");

    stab_dur_ = 5.;
    end_time_ = 1000.;
    ctrl_start_time_ = 0.;

    des_jacc_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

    Kp_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    Kd_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

    ini_com_pos_ = Eigen::VectorXd::Zero(3);
    ini_com_vel_ = Eigen::VectorXd::Zero(3);
    goal_com_pos_ = Eigen::VectorXd::Zero(3);

    // task
    com_task_ = new BasicTask(robot_, BasicTaskType::COM, 3);
    total_joint_task_ = new BasicTask(robot_, BasicTaskType::JOINT, Draco::n_adof);

    body_ori_task_ = new BasicTask(robot_, BasicTaskType::LINKORI, 3, DracoBodyNode::Torso);
    rfoot_center_rz_xyz_task = new FootRzXYZTask(robot_, DracoBodyNode::rFootCenter);
    lfoot_center_rz_xyz_task = new FootRzXYZTask(robot_, DracoBodyNode::lFootCenter);

    rfoot_line_task = new LineFootTask(robot_, DracoBodyNode::rFootCenter);
    lfoot_line_task = new LineFootTask(robot_, DracoBodyNode::lFootCenter);

    rfoot_front_task = new BasicTask(robot_, BasicTaskType::LINKXYZ, 3, DracoBodyNode::rFootFront);
    rfoot_back_task = new BasicTask(robot_, BasicTaskType::LINKXYZ, 3, DracoBodyNode::rFootBack);
    lfoot_front_task = new BasicTask(robot_, BasicTaskType::LINKXYZ, 3, DracoBodyNode::lFootFront);
    lfoot_back_task = new BasicTask(robot_, BasicTaskType::LINKXYZ, 3, DracoBodyNode::lFootBack);

    ang_momentum_task = new AngularMomentumTask(robot_, 0.001);

    // contact
    rfoot_front_contact_ = new PointContactSpec(robot_, DracoBodyNode::rFootFront, 0.7);
    rfoot_back_contact_ = new PointContactSpec(robot_, DracoBodyNode::rFootBack, 0.7);
    lfoot_front_contact_ = new PointContactSpec(robot_, DracoBodyNode::lFootFront, 0.7);
    lfoot_back_contact_ = new PointContactSpec(robot_, DracoBodyNode::lFootBack, 0.7);

    contact_list_.clear();
    contact_list_.push_back(rfoot_front_contact_);
    contact_list_.push_back(rfoot_back_contact_);
    contact_list_.push_back(lfoot_front_contact_);
    contact_list_.push_back(lfoot_back_contact_);

    dim_contact_ = rfoot_front_contact_->getDim() + lfoot_front_contact_->getDim() +
                   rfoot_back_contact_->getDim() + lfoot_back_contact_->getDim();

    // Some helpful defaults
    // Convex MPC
    // convex_mpc = new CMPC();
    max_fz_ = 1500.0; // Maximum Reaction force on each contact point
    Fd_des_ = Eigen::VectorXd::Zero(dim_contact_);
    Fd_des_filtered_ = Eigen::VectorXd::Zero(dim_contact_);
    alpha_fd_ = 0.9;

    // Footsteps and Reference Trajectory Module
    // Set which contact points from the contact_list_ corresponds to which footstep
    std::vector<int> contact_index_to_side = {DRACO_RIGHT_FOOTSTEP, DRACO_RIGHT_FOOTSTEP,
                                              DRACO_LEFT_FOOTSTEP, DRACO_LEFT_FOOTSTEP};
    reference_trajectory_module_ = new DCMWalkingReferenceTrajectoryModule(contact_index_to_side);

    left_foot_start_ = new DracoFootstep();
    right_foot_start_ = new DracoFootstep();

    // Initialize
    swing_foot_current_.reset(new DracoFootstep());
    swing_start_time_ = 0.0;
    swing_end_ = false;

    // Integration Parameters
     max_joint_vel_ = 2.0;
    velocity_break_freq_ = 25.0;
    max_jpos_error_ = 0.2;
    position_break_freq_ = 20.0;

    // wbc
    std::vector<bool> act_list;
    act_list.resize(robot_->getNumDofs(), true);
    for (int i(0); i < robot_->getNumVirtualDofs(); ++i) act_list[i] = false;

    // IHWBC
    last_control_time_ = -0.001;
    ihwbc = new IHWBC(act_list);
    gamma_old_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

    // Regularization terms should always be the lowest cost. 
    lambda_qddot_ = 1e-8;
    lambda_Fr_ = 1e-16;

    // Relative task weighting
    w_task_rfoot_ = 200;// 1e5;
    w_task_lfoot_ = 200;// 1e5;
    w_task_com_ = 1e-3;
    w_task_body_ = 1e-3;
    w_task_joint_ = 1e-6;

    w_task_ang_momentum_ = 1e-5;

    // Relative reaction force tracking weight compared to tasks
    // Must be high if desired reaction force is nonzero. 
    w_contact_weight_ = 1e-2; 

    // Initialize State Provider
    sp_ = DracoStateProvider::getStateProvider(robot_);
}

DCMWalkCtrl::~DCMWalkCtrl() {
    delete com_task_;
    delete total_joint_task_;

    delete left_foot_start_;
    delete right_foot_start_;

    delete rfoot_front_contact_;
    delete rfoot_back_contact_;
    delete lfoot_front_contact_;
    delete lfoot_back_contact_;
}

double DCMWalkCtrl::clamp_value(double in, double min, double max){
    if (in >= max){
        return max;
    }else if (in <= min){
        return min;
    }else{
        return in;
    }
}

double DCMWalkCtrl::computeAlphaGivenBreakFrequency(double hz, double dt){
    double omega = 2.0 * M_PI * hz;
    double alpha = (1.0 - (omega*dt/2.0)) / (1.0 + (omega*dt/2.0));
    alpha = clamp_value(alpha, 0.0, 1.0);
    return alpha;
}

void DCMWalkCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;
    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

    // Get trajectory state
    ctrl_state_ =  reference_trajectory_module_->getState(state_machine_time_);

    // Sotre the desired DCM and r_vrp references
    if (state_machine_time_ >= walk_start_time_){
        ((DCMWalkingReferenceTrajectoryModule*)reference_trajectory_module_)->dcm_reference.get_ref_dcm(state_machine_time_, sp_->dcm_des);    
        ((DCMWalkingReferenceTrajectoryModule*)reference_trajectory_module_)->dcm_reference.get_ref_dcm_vel(state_machine_time_, sp_->dcm_vel_des);    
        ((DCMWalkingReferenceTrajectoryModule*)reference_trajectory_module_)->dcm_reference.get_ref_r_vrp(state_machine_time_, sp_->r_vrp_des);    
    }

    // Setup the contacts
    contact_setup();

    _Xdes_setup();

    // Setup the tasks and compute torque from IHWBC
    task_setup();
    // clock_.start();
    _compute_torque_ihwbc(gamma);
    // printf("time: %f\n", clock_.stop());
    // Store the desired feed forward torque command 


    double alphaTau = (1.0 - computeAlphaGivenBreakFrequency(100.0, ihwbc_dt_));   
    // gamma_old_ = gamma*alphaTau + (1.0 - alphaTau)*gamma_old_;
    gamma_old_ = gamma;

    // Send the Commands
    for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
        ((DracoCommand*)_cmd)->jtrq[i] = gamma_old_[i];
        ((DracoCommand*)_cmd)->q[i] = sp_->des_jpos[i];
        ((DracoCommand*)_cmd)->qdot[i] = sp_->des_jvel[i];
    }

    _PostProcessing_Command();        

    //TODO:
    // if prev_ctrl_state was swing and now we are in double support, set swing_end to true
    if (((prev_ctrl_state_ == DRACO_STATE_RLS) || (prev_ctrl_state_ == DRACO_STATE_LLS)) && (ctrl_state_ == DRACO_STATE_DS)){
        swing_end_ = true;
    }


    // change stance when we enter the swing phase right leg swing
    if ((prev_ctrl_state_ == DRACO_STATE_DS) && (ctrl_state_ == DRACO_STATE_RLS)){
        std::cout << "stance change to left foot!" << std::endl;
        sp_->stance_foot = "lFoot";
    // left leg swing
    }else if ((prev_ctrl_state_ == DRACO_STATE_DS) && (ctrl_state_ == DRACO_STATE_LLS)){
         std::cout << "stance change to right foot!" << std::endl;
        sp_->stance_foot = "rFoot";
    }

    // Store  ctrl_state_ as previous.
    prev_ctrl_state_ = ctrl_state_;
}

void DCMWalkCtrl::references_setup(){
    // Prepare containers
    Eigen::Vector3d x_com_start;
    Eigen::Quaterniond x_ori_start;
    double init_roll(sp_->q[5]), init_pitch(sp_->q[4]), init_yaw(sp_->q[3]);

    // Get current CoM
    x_com_start = sp_->com_pos;
    x_com_start[2] = target_com_height_;
    // Get current Ori
    x_ori_start = myUtils::EulerZYXtoQuat(init_roll, init_pitch, init_yaw);
    // Get current starting left and right footstep configuration

    Eigen::Vector3d lfoot_pos = robot_->getBodyNodeCoMIsometry(DracoBodyNode::lFootCenter).translation();
    Eigen::Quaterniond lfoot_ori(robot_->getBodyNodeCoMIsometry(DracoBodyNode::lFootCenter).linear());
    left_foot_start_->setPosOriSide(lfoot_pos, lfoot_ori, DRACO_LEFT_FOOTSTEP);

    Eigen::Vector3d rfoot_pos = robot_->getBodyNodeCoMIsometry(DracoBodyNode::rFootCenter).translation();
    Eigen::Quaterniond rfoot_ori(robot_->getBodyNodeCoMIsometry(DracoBodyNode::rFootCenter).linear());
    right_foot_start_->setPosOriSide(rfoot_pos, rfoot_ori, DRACO_RIGHT_FOOTSTEP);

    // Set references initial condition
    reference_trajectory_module_->setStartingConfiguration(x_com_start,
                                                           x_ori_start,
                                                           *left_foot_start_,
                                                           *right_foot_start_);

    std::cout << "setting references once!" << std::endl;
    myUtils::pretty_print(x_com_start, std::cout, "x_com_start");
    myUtils::pretty_print(x_ori_start, std::cout, "x_ori_start");
    left_foot_start_->printInfo();
    right_foot_start_->printInfo();

    double factor = 2.5;
    // Set desired footstep landing locations
    // Eigen::Vector3d foot_translate(0.11, 0.0, 0.0); // need to set icp to 5.0 when moving forward
    // Eigen::Quaterniond foot_rotate( Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) );
    // double swing_time_in = 0.5;
    // factor = 1.0;

    // Eigen::Vector3d foot_translate(0.05, 0.0, 0.0); // need to set icp to 5.0 when moving forward
    // Eigen::Quaterniond foot_rotate( Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) );
    // double swing_time_in = 0.3;
    // factor = 1.0;

    Eigen::Vector3d foot_translate(-0.125, 0.0, 0.0);
    Eigen::Quaterniond foot_rotate( Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) );
    double swing_time_in = 0.5;
    factor = 1.0;

    // Eigen::Vector3d foot_translate(0.0, -0.075, 0.0);
    // Eigen::Quaterniond foot_rotate( Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) );
    // double swing_time_in = 0.4;
    // factor = 1.0;

    // Eigen::Vector3d foot_translate(0.0, -0.125, 0.0);
    // Eigen::Quaterniond foot_rotate( Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) );
    // double swing_time_in = 0.4;
    // factor = 1.0;

    // Eigen::Vector3d foot_translate(0.0, 0.0, 0.0);
    // Eigen::Quaterniond foot_rotate( Eigen::AngleAxisd(-M_PI/8.0, Eigen::Vector3d::UnitZ()) );
    // double swing_time_in = 0.4;
    // factor = 1.0;

    // Eigen::Vector3d foot_translate(0.0, 0.0, 0.0);
    // Eigen::Quaterniond foot_rotate( Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) );
    // double swing_time_in = 0.4;
    // factor = 1.0;

    DracoFootstep rfootstep_1; // take a rightfootstep
    rfootstep_1.setPosOriSide(foot_rotate.toRotationMatrix()*(right_foot_start_->position) + foot_translate, 
                              foot_rotate*right_foot_start_->orientation, 
                              DRACO_RIGHT_FOOTSTEP);

    DracoFootstep lfootstep_1; // take a left footstep
    lfootstep_1.setPosOriSide(foot_rotate.toRotationMatrix()*(left_foot_start_->position) + foot_translate*factor, 
                              foot_rotate*left_foot_start_->orientation, 
                              DRACO_LEFT_FOOTSTEP);

    // Compute landing locations from the current midfoot location
    // translate from the nominal midfoot configuration.
    double nominal_midfoot_distance = 0.33;
    DracoFootstep current_midfoot; current_midfoot.computeMidfeet(*left_foot_start_, *right_foot_start_, current_midfoot);
    DracoFootstep midfoot_land; 
    DracoFootstep rfoot_proxy ; 
    DracoFootstep lfoot_proxy;

    midfoot_land.setPosOriSide(foot_rotate.toRotationMatrix()*current_midfoot.position + foot_translate,
                           foot_rotate*current_midfoot.orientation,
                           DRACO_MID_FOOTSTEP);

    rfoot_proxy.setPosOriSide(midfoot_land.position + midfoot_land.R_ori*Eigen::Vector3d(0, -nominal_midfoot_distance/2.0, 0.0),
                           midfoot_land.orientation,
                           DRACO_RIGHT_FOOTSTEP);
    lfoot_proxy.setPosOriSide(midfoot_land.position + midfoot_land.R_ori*Eigen::Vector3d(0, nominal_midfoot_distance/2.0, 0.0),
                           midfoot_land.orientation,
                           DRACO_LEFT_FOOTSTEP);
    rfootstep_1 = rfoot_proxy;
    lfootstep_1 = lfoot_proxy;



    double double_contact_time_in = 0.05;
    double contact_transition_time_in = 0.2;

    double swing_height_in = 0.05;

    rfootstep_1.setWalkingParams(double_contact_time_in,
                                 contact_transition_time_in,
                                 swing_time_in,
                                 swing_height_in);

    lfootstep_1.setWalkingParams(double_contact_time_in,
                                 contact_transition_time_in,
                                 swing_time_in,
                                 swing_height_in);

    DracoFootstep rfootstep_2; // take a rightfootstep
    rfootstep_2.setPosOriSide(rfootstep_1.position + foot_translate*factor, 
                              foot_rotate*rfootstep_1.orientation, 
                              DRACO_RIGHT_FOOTSTEP);

    DracoFootstep lfootstep_2; // take a leftfootstep
    lfootstep_2.setPosOriSide(lfootstep_1.position + foot_translate*factor, 
                              foot_rotate*lfootstep_1.orientation, 
                              DRACO_LEFT_FOOTSTEP);


    // Clear then add footsteps to the list.
    desired_footstep_list_.clear();
    desired_footstep_list_.push_back(rfootstep_1);
    desired_footstep_list_.push_back(lfootstep_1);
    // desired_footstep_list_.push_back(rfootstep_2);
    // desired_footstep_list_.push_back(lfootstep_2);

    for(int i = 0; i < desired_footstep_list_.size(); i++){
        printf("Step %i:\n", i);
        desired_footstep_list_[i].printInfo();
    }

    // Update the reference trajectory module
    ((DCMWalkingReferenceTrajectoryModule*)reference_trajectory_module_)->dcm_reference.setCoMHeight(target_com_height_);
    ((DCMWalkingReferenceTrajectoryModule*)reference_trajectory_module_)->dcm_reference.t_ss = swing_time_in;
    reference_trajectory_module_->setFootsteps(walk_start_time_, desired_footstep_list_);
    end_time_ = ((DCMWalkingReferenceTrajectoryModule*)reference_trajectory_module_)->dcm_reference.get_total_trajectory_time();
    end_time_ += walk_start_time_;


}

void DCMWalkCtrl::_Xdes_setup(){
    int n = 13; 
    Xdes_ = Eigen::VectorXd::Zero(n*1); // Create the desired state vector evolution

    double magnitude = 0.0;
    double T = 1.0;
    double freq = 1/T;
    double omega = 2 * M_PI * freq;

    Eigen::Vector3d ypr = myUtils::QuatToEulerZYX(ini_ori_);

    // Set Targets to current 
    // Set Desired Orientation
    Xdes_[0] = ypr[2]; // Desired Roll
    Xdes_[1] = ypr[1]; // Desired Pitch
    Xdes_[2] = ypr[0]; // Desired Yaw

    // ----------------------------------------------------------------
    // Set CoM Position -----------------------------------------------
    Xdes_[3] = goal_com_pos_[0];
    Xdes_[4] = goal_com_pos_[1];
    Xdes_[5] = goal_com_pos_[2];    

    // Set CoM Velocity -----------------------------------------------
    Xdes_[9] = 0.0;
    Xdes_[10] = 0.0;
    Xdes_[11] = 0.0;

}

void DCMWalkCtrl::_compute_torque_ihwbc(Eigen::VectorXd& gamma) {
    // When Fd is nonzero, we need to make the contact weight large if we want to trust the output of the mpc
    // 1e-2/(robot_->getRobotMass()*9.81);
    double local_w_contact_weight = w_contact_weight_/(robot_->getRobotMass()*9.81);

    // Modify Rotor Inertia
    Eigen::MatrixXd A_rotor = A_;
    // for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
    //     A_rotor(i + robot_->getNumVirtualDofs(),
    //             i + robot_->getNumVirtualDofs()) += sp_->rotor_inertia[i];
    // }
    Eigen::MatrixXd A_rotor_inv = A_rotor.inverse();

    // Enable Torque Limits
    ihwbc->enableTorqueLimits(true);
    double tau_lim = 100.0;    
    Eigen::VectorXd tau_min = -tau_lim*Eigen::VectorXd::Ones(Draco::n_adof);
    Eigen::VectorXd tau_max = tau_lim*Eigen::VectorXd::Ones(Draco::n_adof);
    ihwbc->setTorqueLimits(tau_min, tau_max);

    // Set QP weights
    ihwbc->setQPWeights(w_task_heirarchy_, local_w_contact_weight);
    ihwbc->setRegularizationTerms(lambda_qddot_, lambda_Fr_);

    // QP dec variable results
    Eigen::VectorXd qddot_res;
    Eigen::VectorXd Fr_res;

    // Update and solve QP
    ihwbc->updateSetting(A_rotor, A_rotor_inv, coriolis_, grav_);

    // Desired Reaction Forces
    ihwbc->solve(task_list_, contact_list_, Fd_des_filtered_, tau_cmd_, qddot_cmd_);

    ihwbc->getQddotResult(qddot_res);
    ihwbc->getFrResult(Fr_res);

    // Compute desired joint position and velocities for the low-level
    q_current_ = sp_->q; //robot_->getQ();
    qdot_current_ = sp_->qdot; //robot_->getQdot();

    Eigen::VectorXd ac_qdot_current = qdot_current_.tail(robot_->getNumActuatedDofs());
    Eigen::VectorXd ac_q_current = q_current_.tail(robot_->getNumActuatedDofs());

    qdot_des_ = ac_qdot_current + qddot_cmd_*ihwbc_dt_;                  
    q_des_    = ac_q_current + ac_qdot_current*ihwbc_dt_ + 
                0.5*qddot_cmd_*ihwbc_dt_*ihwbc_dt_; 


    gamma = tau_cmd_;

    // Integrate qddot for qdot and q
    // Integrate Joint velocities
    Eigen::VectorXd qdot_des_ref = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    double alphaVelocity = computeAlphaGivenBreakFrequency(velocity_break_freq_, ihwbc_dt_);
    // // Decay desired velocity to 0.0
    sp_->des_jvel = (sp_->des_jvel)*alphaVelocity + (1.0 - alphaVelocity )*qdot_des_ref;
    sp_->des_jvel += (qddot_cmd_*ihwbc_dt_);
    // Clamp Joint Velocity Values
    for(int i = 0; i < sp_->des_jvel.size(); i++){
        sp_->des_jvel[i] = clamp_value(sp_->des_jvel[i], -max_joint_vel_, max_joint_vel_);
    }
    // myUtils::pretty_print(sp_->des_jvel, std::cout, "sp_->des_jvel");    

    // // Integrate Joint Positions
    Eigen::VectorXd q_des_ref = q_current_.tail(robot_->getNumActuatedDofs());
    double alphaPosition = computeAlphaGivenBreakFrequency(position_break_freq_, ihwbc_dt_);
    sp_->des_jpos = sp_->des_jpos*alphaPosition + (1.0 - alphaPosition)*q_des_ref;
    sp_->des_jpos += (sp_->des_jvel*ihwbc_dt_);
    // Clamp desired joint position to maximum error
    for(int i = 0; i < sp_->des_jpos.size(); i++){
        sp_->des_jpos[i] = clamp_value(sp_->des_jpos[i], q_des_ref[i]-max_jpos_error_, q_des_ref[i]+max_jpos_error_);
    }

    // Store desired qddot
    sp_->qddot_cmd = qddot_res;    

    // Store desired reaction force data
    sp_->filtered_rf = Fd_des_;
    sp_->reaction_forces = Fr_res;

}

void DCMWalkCtrl::task_setup() {
    // Disable MPC
    double des_roll = Xdes_[0];
    double des_pitch = Xdes_[1];
    double des_yaw = Xdes_[2];

    double des_pos_x = Xdes_[3];
    double des_pos_y = Xdes_[4];
    double des_pos_z = Xdes_[5];

    double des_rx_rate = Xdes_[6];
    double des_ry_rate = Xdes_[7];
    double des_rz_rate = Xdes_[8];

    double des_vel_x = Xdes_[9];
    double des_vel_y = Xdes_[10];
    double des_vel_z = Xdes_[11];

    double des_rx_acc = 0.0; 
    double des_ry_acc = 0.0; 
    double des_rz_acc = 0.0; 

    double des_acc_x = 0.0; 
    double des_acc_y = 0.0; 
    double des_acc_z = 0.0; 

    Eigen::Vector3d com_pos_ref, com_vel_ref;
    Eigen::Vector3d dcm_ref, dcm_vel_ref;
    Eigen::Quaterniond ori_ref;
    Eigen::Vector3d euler_yaw_pitch_roll;

    // Angular velocity and acceleration references
    Eigen::Vector3d ang_vel_ref, ang_acc_ref;
    Eigen::Vector3d com_acc_des, com_vel_des, com_pos_des;
    ang_vel_ref.setZero(); ang_acc_ref.setZero();

    if (state_machine_time_ >= walk_start_time_){
        reference_trajectory_module_->getMPCRefComPosandVel(state_machine_time_, com_pos_ref, com_vel_ref);
        reference_trajectory_module_->getMPCRefQuatAngVelAngAcc(state_machine_time_, ori_ref, ang_vel_ref, ang_acc_ref);            

        euler_yaw_pitch_roll = myUtils::QuatToEulerZYX(ori_ref);
        des_roll = euler_yaw_pitch_roll[2]; // Desired Roll
        des_pitch = euler_yaw_pitch_roll[1]; // Desired Pitch
        des_yaw = euler_yaw_pitch_roll[0]; // Desired Yaw            

        des_pos_x = sp_->com_pos[0]; //com_pos_ref[0]; 
        des_pos_y = sp_->com_pos[1]; //com_pos_ref[1]; 
        des_pos_z = goal_com_pos_[2]; 

        des_rx_rate = ang_vel_ref[0]; 
        des_ry_rate = ang_vel_ref[1]; 
        des_rz_rate = ang_vel_ref[2]; 

        des_vel_x = sp_->com_vel[0]; //com_vel_ref[0]; 
        des_vel_y = sp_->com_vel[1]; //com_vel_ref[1]; 
        des_vel_z = com_vel_ref[2]; 

        des_rx_acc = ang_acc_ref[0]; 
        des_ry_acc = ang_acc_ref[1]; 
        des_rz_acc = ang_acc_ref[2]; 

        // des_acc_x = 0.0; 
        // des_acc_y = 0.0; 
        // des_acc_z = 0.0;

        double gravity = 9.81;
        double com_height = goal_com_pos_[2]; 
        double omega_o = std::sqrt(gravity/com_height);

        // Current ICP 
        Eigen::VectorXd r_ic = sp_->dcm.head(2);
        // Desired ICP
        Eigen::VectorXd r_id = sp_->dcm_des.head(2);
        // Desired ICP Velocity
        Eigen::VectorXd rdot_id = sp_->dcm_vel_des.head(2);

        // Desired CMP
        Eigen::VectorXd r_icp_error = (r_ic - r_id);
        // icp_acc_error_ += (r_icp_error*ihwbc_dt_);
        // icp_acc_error_[0] = clamp_value(icp_acc_error_[0], -icp_sat_error_, icp_sat_error_);
        // icp_acc_error_[1] = clamp_value(icp_acc_error_[1], -icp_sat_error_, icp_sat_error_);

        double kp_ic = 20.0;
        Eigen::VectorXd r_CMP_d = r_ic - rdot_id/omega_o + kp_ic * (r_icp_error);// + ki_ic_*icp_acc_error_;
        // Eigen::VectorXd r_CMP_d = r_ic + kp_ic * (r_icp_error);

        // myUtils::pretty_print(r_icp_error, std::cout, "r_icp_error");
        // myUtils::pretty_print(icp_acc_error_, std::cout, "icp_acc_error_");
        // myUtils::pretty_print(r_icp_error, std::cout, "r_icp_error");

        // Desired Linear Momentum / Acceleration task:
        com_acc_des.head(2) = (gravity/com_height)*( sp_->com_pos.head(2) - r_CMP_d);
        des_acc_x = com_acc_des[0]; 
        des_acc_y = com_acc_des[1]; 
        des_acc_z = 0.0;

        sp_->com_pos_des = com_pos_ref;
        sp_->com_pos_des[2] = goal_com_pos_[2];

        sp_->com_vel_des = com_vel_ref;
        sp_->com_vel_des[2] = 0.0;

    }else{
        sp_->com_pos_des[0] = des_pos_x;
        sp_->com_pos_des[1] = des_pos_y;
        sp_->com_pos_des[2] = des_pos_z;
        sp_->com_vel_des[0] = des_vel_x;
        sp_->com_vel_des[1] = des_vel_y;
        sp_->com_vel_des[2] = des_vel_z;

    }

    com_acc_des.setZero();
    com_pos_des[0] = des_pos_x;
    com_pos_des[1] = des_pos_y;
    com_pos_des[2] = des_pos_z;

    com_vel_des[0] = des_vel_x;
    com_vel_des[1] = des_vel_y;
    com_vel_des[2] = des_vel_z;


    com_acc_des[0] = des_acc_x; 
    com_acc_des[1] = des_acc_y; 
    com_acc_des[2] = des_acc_z; 

    // std::cout << "com_pos_des = " << com_pos_des.transpose() << std::endl;
    // std::cout << "com_vel_des = " << com_vel_des.transpose() << std::endl;
    // std::cout << "com_acc_des = " << com_acc_des.transpose() << std::endl;    

    com_task_->updateTask(com_pos_des, com_vel_des, com_acc_des);

    // =========================================================================
    // Body Ori Task
    // =========================================================================
    Eigen::VectorXd body_ori_des(4); body_ori_des.setZero();
    Eigen::VectorXd body_ori_vel_des(3); body_ori_vel_des.setZero();    
    Eigen::VectorXd body_ori_acc_des(3); body_ori_acc_des.setZero();
    Eigen::Quaternion<double> body_ori_des_quat(1, 0, 0, 0);

    body_ori_des_quat = myUtils::EulerZYXtoQuat(des_roll, des_pitch, des_yaw);
    body_ori_des[0] = body_ori_des_quat.w();
    body_ori_des[1] = body_ori_des_quat.x();
    body_ori_des[2] = body_ori_des_quat.y();
    body_ori_des[3] = body_ori_des_quat.z();

    // body_ori_vel_des = myUtils::EulerZYXRatestoAngVel(des_roll, des_pitch, des_yaw,
    //                                                   des_rx_rate, des_ry_rate, des_rz_rate);

    body_ori_vel_des = Eigen::Vector3d(des_rx_rate, des_ry_rate, des_rz_rate);        
    body_ori_acc_des = Eigen::Vector3d(des_rx_acc, des_ry_acc, des_rz_acc);
    body_ori_task_->updateTask(body_ori_des, body_ori_vel_des, body_ori_acc_des);

    // =========================================================================
    // Foot Center Tasks
    // =========================================================================

    //TODO: Fix foot center tasks to conrol, x,y,z, rx, rz

    Eigen::VectorXd rfoot_pos_des(7); rfoot_pos_des.setZero();
    Eigen::VectorXd lfoot_pos_des(7); lfoot_pos_des.setZero();
    Eigen::VectorXd foot_vel_des(6); foot_vel_des.setZero();    
    Eigen::VectorXd foot_acc_des(6); foot_acc_des.setZero();

    Eigen::Quaternion<double> rfoot_ori_act(robot_->getBodyNodeCoMIsometry(DracoBodyNode::rFootCenter).linear());
    Eigen::Quaternion<double> lfoot_ori_act(robot_->getBodyNodeCoMIsometry(DracoBodyNode::lFootCenter).linear());

    rfoot_pos_des[0] = rfoot_ori_act.w();
    rfoot_pos_des[1] = rfoot_ori_act.x();
    rfoot_pos_des[2] = rfoot_ori_act.y();
    rfoot_pos_des[3] = rfoot_ori_act.z();
    rfoot_pos_des.tail(3) = robot_->getBodyNodeCoMIsometry(DracoBodyNode::rFootCenter).translation();

    lfoot_pos_des[0] = lfoot_ori_act.w();
    lfoot_pos_des[1] = lfoot_ori_act.x();
    lfoot_pos_des[2] = lfoot_ori_act.y();
    lfoot_pos_des[3] = lfoot_ori_act.z();
    lfoot_pos_des.tail(3) = robot_->getBodyNodeCoMIsometry(DracoBodyNode::lFootCenter).translation();

    rfoot_center_rz_xyz_task->updateTask(rfoot_pos_des, foot_vel_des, foot_acc_des);
    lfoot_center_rz_xyz_task->updateTask(lfoot_pos_des, foot_vel_des, foot_acc_des);

    Eigen::VectorXd foot_pos_d(3); foot_pos_d.setZero();
    Eigen::VectorXd foot_vel_d(3); foot_vel_d.setZero();
    Eigen::VectorXd foot_acc_d(3); foot_acc_d.setZero();

    // Compute swing foot trajectories --------------------------------------------------------------
    Eigen::Vector3d f_pos, f_vel, f_acc;
    Eigen::Quaterniond f_ori;       
    Eigen::Vector3d f_ori_vel, f_ori_acc;
    Eigen::Vector3d toe_pos;
    Eigen::Vector3d heel_pos;
    // Check if we are in swing. If so, perform a swing foot trajectory
    if ((ctrl_state_ == DRACO_STATE_RLS) || ctrl_state_ == DRACO_STATE_LLS){
        // if the previous state was a double support, we have to construct a trajectory.
        if (prev_ctrl_state_ == DRACO_STATE_DS){
            compute_swing_foot_trajectory();            
        }
      
        // Compute progression variable
        double s = (state_machine_time_ - swing_start_time_)/swing_foot_current_->swing_time;
        // swing foot trajectory is split into two. so divide by 2.0 first

        // 0.0 <= s < 1.0 use the first trajectory
        // 1.0 <= s < 2.0 use the second trajectory
        if (s <= 0.5){
            pos_traj_to_use = foot_pos_traj_init_to_mid_;
            // scale back to 1.0
            s = 2.0*s;
        }else{
            pos_traj_to_use = foot_pos_traj_mid_to_end_;    
            // scale back to 1.0 after the offset
            s = 2.0*(s - 0.5);
        }

        // Get position, orientation and its derivatives
        f_pos = pos_traj_to_use->evaluate(s);
        f_vel = pos_traj_to_use->evaluateFirstDerivative(s);
        f_acc = pos_traj_to_use->evaluateSecondDerivative(s);
        foot_ori_trajectory->evaluate(s, f_ori);
        foot_ori_trajectory->getAngularVelocity(s, f_ori_vel);
        foot_ori_trajectory->getAngularAcceleration(s, f_ori_acc);

        // printf("s: %0.3f. swing_time: %0.3f, swing_height: %0.3f \n", s, swing_foot_current_->swing_time, swing_foot_current_->swing_height);
        // myUtils::pretty_print(f_pos, std::cout, "  f_pos");
        // myUtils::pretty_print(f_vel, std::cout, "  f_vel");
        // myUtils::pretty_print(f_acc, std::cout, "  f_acc");

        // myUtils::pretty_print(f_ori, std::cout, "  f_ori");
        // myUtils::pretty_print(f_ori_vel, std::cout, "  f_ori_vel");
        // myUtils::pretty_print(f_ori_acc, std::cout, "  f_ori_acc");

        // Set the current swing foot position and orientation
        swing_foot_current_->setPosOri(f_pos, f_ori);

        // Grab the toe and heel position
        toe_pos = swing_foot_current_->getToePosition();
        heel_pos = swing_foot_current_->getHeelPosition();
    }

    // Use Swing foot trajectories if we are in swing
    if (ctrl_state_ == DRACO_STATE_RLS){
        // Set Point Tasks
        rfoot_front_task->updateTask(toe_pos, f_vel, f_acc);
        rfoot_back_task->updateTask(heel_pos, f_vel, f_acc);        

        // Set positions
        rfoot_pos_des[0] = f_ori.w(); rfoot_pos_des[1] = f_ori.x();      
        rfoot_pos_des[2] = f_ori.y(); rfoot_pos_des[3] = f_ori.z();
        rfoot_pos_des.tail(3) = f_pos;
        // Set velocities
        foot_vel_des.head(3) = f_ori_vel;
        foot_vel_des.tail(3) = f_vel;
        // Set Accelerations
        foot_acc_des.head(3) = f_ori_acc;
        foot_acc_des.tail(3) = f_acc;
        // Set Line Task
        rfoot_line_task->updateTask(rfoot_pos_des, foot_vel_des, foot_acc_des);
    }else{
        // Set Point Tasks
        foot_pos_d =  robot_->getBodyNodeCoMIsometry(DracoBodyNode::rFootFront).translation();   
        rfoot_front_task->updateTask(foot_pos_d, foot_vel_d, foot_acc_d);
        foot_pos_d = robot_->getBodyNodeCoMIsometry(DracoBodyNode::rFootBack).translation();
        rfoot_back_task->updateTask(foot_pos_d, foot_vel_d, foot_acc_d);        
        // Set Line Task
        rfoot_line_task->updateTask(rfoot_pos_des, foot_vel_des, foot_acc_des);
    }

    if (ctrl_state_ == DRACO_STATE_LLS){
        // Set Point Tasks
        lfoot_front_task->updateTask(toe_pos, f_vel, f_acc);
        lfoot_back_task->updateTask(heel_pos, f_vel, f_acc);

        // Set Postitions
        lfoot_pos_des[0] = f_ori.w(); lfoot_pos_des[1] = f_ori.x();      
        lfoot_pos_des[2] = f_ori.y(); lfoot_pos_des[3] = f_ori.z();
        lfoot_pos_des.tail(3) = f_pos;
        // Set velocities
        foot_vel_des.head(3) = f_ori_vel;
        foot_vel_des.tail(3) = f_vel;
        // Set Accelerations
        foot_acc_des.head(3) = f_ori_acc;
        foot_acc_des.tail(3) = f_acc;
        // Set Line Task
        lfoot_line_task->updateTask(lfoot_pos_des, foot_vel_des, foot_acc_des);

    }else{
        // Set Point Tasks
        foot_pos_d = robot_->getBodyNodeCoMIsometry(DracoBodyNode::lFootFront).translation();
        lfoot_front_task->updateTask(foot_pos_d, foot_vel_d, foot_acc_d);
        foot_pos_d = robot_->getBodyNodeCoMIsometry(DracoBodyNode::lFootBack).translation();
        lfoot_back_task->updateTask(foot_pos_d, foot_vel_d, foot_acc_d);        
        // Set Line Task
        lfoot_line_task->updateTask(lfoot_pos_des, foot_vel_des, foot_acc_des);
    }

    // std::cout << "  rFootCenter = " << robot_->getBodyNodeCoMIsometry(DracoBodyNode::rFootCenter).translation().transpose() << std::endl;  

    // =========================================================================
    // Angular Momentum Task
    // =========================================================================
    Eigen::Vector3d des_ang_vel(des_rx_rate, des_ry_rate, des_rz_rate);
    Eigen::Vector3d des_ang_acc(des_rx_acc, des_ry_acc, des_rz_acc);

    Eigen::Vector3d des_ang_momentum; des_ang_momentum.setZero();
    Eigen::Vector3d des_ang_momentum_rate; des_ang_momentum_rate.setZero();
    Eigen::MatrixXd I_g = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd I_g_tmp = robot_->getCentroidInertia();

    I_g = I_g_tmp.block(0, 0, 3, 3);
    des_ang_momentum = I_g*des_ang_vel;
    des_ang_momentum_rate = I_g*des_ang_acc;
    ang_momentum_task->updateTask(Eigen::Vector3d(0,0,0), des_ang_momentum, des_ang_momentum_rate);

    // =========================================================================
    // Joint Pos Task
    // =========================================================================
    Eigen::VectorXd jpos_des = jpos_ini_;
    Eigen::VectorXd jvel_des(Draco::n_adof);
    jvel_des.setZero();
    Eigen::VectorXd jacc_des(Draco::n_adof);
    jacc_des.setZero();
    total_joint_task_->updateTask(jpos_des, jvel_des, jacc_des);

    // =========================================================================
    // Task List Update
    // =========================================================================

    // double t_transition = reference_trajectory_module_->reaction_force_schedule_ptr->getTransitionVariable(0, state_machine_time_);
 
    // double w_contact_transition = t_transition*w_task_rfoot_;
    // double w_swing_transition = (1-t_transition)*1e-1;

    // task_list_.push_back(com_task_);
    // task_list_.push_back(body_ori_task_);

    // task_list_.push_back(rfoot_line_task);
    // task_list_.push_back(lfoot_line_task); 

    // task_list_.push_back(rfoot_front_task);
    // task_list_.push_back(rfoot_back_task);            
    // task_list_.push_back(lfoot_front_task);
    // task_list_.push_back(lfoot_back_task);

    // w_task_heirarchy_ = Eigen::VectorXd::Zero(task_list_.size());
    // w_task_heirarchy_[0] = w_task_com_; // COM
    // w_task_heirarchy_[1] = w_task_body_; // body ori

    // w_task_heirarchy_[2] = w_swing_transition; // rfoot swing
    // w_task_heirarchy_[3] = w_swing_transition; // lfoot swing

    // w_task_heirarchy_[4] = t_transition*w_task_rfoot_; // rfoot contact
    // w_task_heirarchy_[5] = t_transition*w_task_rfoot_; // rfoot contact
    // w_task_heirarchy_[6] = t_transition*w_task_lfoot_; // rfoot contact
    // w_task_heirarchy_[7] = t_transition*w_task_lfoot_; // rfoot contact


    task_list_.push_back(com_task_);
    task_list_.push_back(body_ori_task_);

    if (ctrl_state_ == DRACO_STATE_RLS){
        task_list_.push_back(rfoot_line_task);
        task_list_.push_back(lfoot_front_task);
        task_list_.push_back(lfoot_back_task);
    }else if (ctrl_state_ == DRACO_STATE_LLS){
        task_list_.push_back(lfoot_line_task);            
        task_list_.push_back(rfoot_front_task);
        task_list_.push_back(rfoot_back_task);            
    }else{
        task_list_.push_back(rfoot_front_task);
        task_list_.push_back(rfoot_back_task);
        task_list_.push_back(lfoot_front_task);
        task_list_.push_back(lfoot_back_task);
    }
    // task_list_.push_back(ang_momentum_task);
    // task_list_.push_back(total_joint_task_);

    w_task_heirarchy_ = Eigen::VectorXd::Zero(task_list_.size());
    w_task_heirarchy_[0] = w_task_com_; // COM
    w_task_heirarchy_[1] = w_task_body_; // body ori

    if (ctrl_state_ == DRACO_STATE_RLS){
        w_task_heirarchy_[2] = 1e-2; // rfoot swing
        w_task_heirarchy_[3] = w_task_lfoot_; // lfoot contact
        w_task_heirarchy_[4] = w_task_lfoot_; // lfoot contact
    }else if (ctrl_state_ == DRACO_STATE_LLS){
        w_task_heirarchy_[2] = 1e-2; // lfoot swing
        w_task_heirarchy_[3] = w_task_rfoot_; // rfoot contact
        w_task_heirarchy_[4] = w_task_rfoot_; // rfoot contact
    }else{
        w_task_heirarchy_[2] = w_task_rfoot_; // rfoot contact
        w_task_heirarchy_[3] = w_task_rfoot_; // lfoot contact
        w_task_heirarchy_[4] = w_task_lfoot_; // lfoot contact
        w_task_heirarchy_[5] = w_task_lfoot_; // lfoot contact
    }
    // w_task_heirarchy_[task_list_.size() - 1] = 1e-2; // angular momentum 
    // w_task_heirarchy_[task_list_.size() - 1] = w_task_joint_; // joint task

}

void DCMWalkCtrl::compute_swing_foot_trajectory(){
    DracoFootstep swing_init_foot;
    DracoFootstep swing_midfoot;
    DracoFootstep swing_land_foot;

    int footstep_index;
    // Compute the trajectories if we are actually in swing
    reference_trajectory_module_->whichFootstepIndexInSwing(state_machine_time_, footstep_index);
    // Get the swing foot landing position
    swing_land_foot = desired_footstep_list_[footstep_index];
    swing_init_foot = swing_land_foot; // get params of the swing land foot.

    // Initialize the swing foot
    int foot_side;
    if (swing_land_foot.robot_side == DRACO_RIGHT_FOOTSTEP){
        foot_side = DracoBodyNode::rFootCenter;
    }else if (swing_land_foot.robot_side == DRACO_LEFT_FOOTSTEP){
        foot_side = DracoBodyNode::lFootCenter;
    }
    Eigen::Vector3d local_foot_pos = robot_->getBodyNodeCoMIsometry(foot_side).translation();
    Eigen::Quaterniond local_foot_ori(robot_->getBodyNodeCoMIsometry(foot_side).linear());
    swing_init_foot.setPosOriSide(local_foot_pos, local_foot_ori, DRACO_RIGHT_FOOTSTEP);               

    // Compute where the foot will be in the middle of the trajectory
    swing_midfoot.computeMidfeet(swing_init_foot, swing_land_foot, swing_midfoot);

    // Compute midfeet boundary conditions
    // Linear velocity at the middle of the swing is the total swing travel over swing time 
    
    double swing_height = swing_land_foot.swing_height;
    double swing_time = swing_land_foot.swing_time;

    Eigen::Vector3d mid_swing_local_foot_pos(0, 0, swing_height);
    Eigen::Vector3d mid_swing_position = swing_midfoot.position + swing_midfoot.R_ori*mid_swing_local_foot_pos;
    Eigen::Vector3d mid_swing_velocity = (swing_land_foot.position - swing_init_foot.position)/swing_time;

    // Construct Position trajectory  
    foot_pos_traj_init_to_mid_.reset(new HermiteCurveVec(swing_init_foot.position, Eigen::Vector3d::Zero(3), 
                                                           mid_swing_position, mid_swing_velocity));

    foot_pos_traj_mid_to_end_.reset(new HermiteCurveVec(mid_swing_position, mid_swing_velocity, 
                                                        swing_land_foot.position, Eigen::Vector3d::Zero(3)));

    // Construct Quaternion trajectory
    Eigen::Vector3d ang_vel_start; ang_vel_start.setZero();
    Eigen::Vector3d ang_vel_end; ang_vel_end.setZero();
    foot_ori_trajectory.reset(new HermiteQuaternionCurve(swing_init_foot.orientation, ang_vel_start,
                                                         swing_land_foot.orientation, ang_vel_end));
    // set initial swing start time
    swing_start_time_ = state_machine_time_;
    // initially set position trajectory to use
    pos_traj_to_use = foot_pos_traj_init_to_mid_;
    // initialize the current swing foot location
    swing_foot_current_->setPosOriSide(swing_init_foot.position, swing_init_foot.orientation, swing_init_foot.robot_side);
    swing_foot_current_->setWalkingParams(swing_init_foot.double_contact_time,
                                          swing_init_foot.contact_transition_time,
                                          swing_init_foot.swing_time,
                                          swing_init_foot.swing_height);
}

void DCMWalkCtrl::contact_setup() {
    rfoot_front_contact_->updateContactSpec();
    rfoot_back_contact_->updateContactSpec();
    lfoot_front_contact_->updateContactSpec();
    lfoot_back_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_front_contact_);
    contact_list_.push_back(rfoot_back_contact_);
    contact_list_.push_back(lfoot_front_contact_);
    contact_list_.push_back(lfoot_back_contact_);

    // Smoothly change maximum Fz for IHWBC
    double smooth_max_fz = max_fz_;//myUtils::smooth_changing(0.0, max_fz_, contact_transition_dur_, state_machine_time_ );
 
    for(int i = 0; i < contact_list_.size(); i++){
        if (state_machine_time_ >= walk_start_time_){
            ((PointContactSpec*)contact_list_[i])->setMaxFz(reference_trajectory_module_->getMaxNormalForce(i, state_machine_time_) );
            // printf("    Contact:%i, force:%0.3f\n", i, reference_trajectory_module_->getMaxNormalForce(i, state_machine_time_));
        }else{
            ((PointContactSpec*)contact_list_[i])->setMaxFz(smooth_max_fz);            
        }

    }
    

 }

void DCMWalkCtrl::firstVisit() {
    std::cout << "[DCM Walk Ctrl] Visit." <<  std::endl;
    // Initialize control state as double support
    ctrl_state_ = DRACO_STATE_DS;
    prev_ctrl_state_ = DRACO_STATE_DS;

    jpos_ini_ = sp_->q.segment(robot_->getNumVirtualDofs(),
                               robot_->getNumActuatedDofs());
    jdot_ini_ = sp_->qdot.segment(robot_->getNumVirtualDofs(),
                                  robot_->getNumActuatedDofs());

    // sp_->des_jpos = jpos_ini_;
    // sp_->des_jvel = jdot_ini_;

    ctrl_start_time_ = sp_->curr_time;

    Eigen::Vector3d com_pos = sp_->com_pos; //robot_->getCoMPosition();
    Eigen::Vector3d com_vel = sp_->est_com_vel ;// robot_->getCoMVelocity();
    Eigen::VectorXd rankle_pos = robot_->getBodyNodeIsometry(DracoBodyNode::rAnkle).translation();
    Eigen::VectorXd lankle_pos = robot_->getBodyNodeIsometry(DracoBodyNode::lAnkle).translation();
    for (int i = 0; i < 3; ++i) {
        ini_com_pos_[i] = com_pos[i];
        ini_com_vel_[i] = com_vel[i];
        goal_com_pos_[i] = (rankle_pos[i] + lankle_pos[i]) / 2.0;
    }
    std::cout << "rankle_pos: " << rankle_pos.transpose() << std::endl;
    std::cout << "lankle_pos: " << lankle_pos.transpose() << std::endl;

    // Set CoM X Goal to 0.0
    // goal_com_pos_[0] = 0.0;
    // goal_com_pos_[2] = target_com_height_;
    // goal_com_pos_ = ini_com_pos_;
    goal_com_pos_ = sp_->com_pos_des;
    goal_com_pos_[2] = target_com_height_;

    double init_roll(sp_->q[5]), init_pitch(sp_->q[4]), init_yaw(sp_->q[3]);
    ini_ori_ = myUtils::EulerZYXtoQuat(init_roll, init_pitch, init_yaw);
    goal_ori_ = ini_ori_;

    std::cout << "Integration Params" << std::endl;
    std::cout << "  Max Joint Velocity: " << max_joint_vel_ << std::endl;
    std::cout << "  Velocity Break Freq : " << velocity_break_freq_ << std::endl;
    std::cout << "  Max Joint Position Error : " << max_jpos_error_ << std::endl;
    std::cout << "  Position Break Freq : " << position_break_freq_ << std::endl;

    std::cout << "IHWBC reaction force alpha:" << alpha_fd_ << std::endl;

    std::cout << "walk_start_time:" << walk_start_time_ << std::endl;

    references_setup();

}

void DCMWalkCtrl::lastVisit() {}

bool DCMWalkCtrl::endOfPhase() {
    if (state_machine_time_ > end_time_) {
        return true;
    }
    return false;
}

void DCMWalkCtrl::ctrlInitialization(const YAML::Node& node) {
    jpos_ini_ = sp_->q.segment(robot_->getNumVirtualDofs(),
                               robot_->getNumActuatedDofs());

    Eigen::VectorXd kp_foot = 100*Eigen::VectorXd::Ones(4); 
    Eigen::VectorXd kd_foot = 10.0*Eigen::VectorXd::Ones(4);

    Eigen::VectorXd com_kp = 50.0*Eigen::VectorXd::Ones(3);
    Eigen::VectorXd com_kd = 5.0*Eigen::VectorXd::Zero(3);

    Eigen::VectorXd kp_body_rpy = 100*Eigen::VectorXd::Ones(3); 
    Eigen::VectorXd kd_body_rpy = 10.0*Eigen::VectorXd::Ones(3);

    Eigen::VectorXd kp_jp = 10*Eigen::VectorXd::Ones(Draco::n_adof); 
    Eigen::VectorXd kd_jp = 0.1*Eigen::VectorXd::Ones(Draco::n_adof);

    try {
        // Integration Parameters
        myUtils::readParameter(node, "max_joint_vel", max_joint_vel_);
        myUtils::readParameter(node, "velocity_break_freq", velocity_break_freq_);
        
        myUtils::readParameter(node, "max_jpos_error", max_jpos_error_);
        myUtils::readParameter(node, "position_break_freq", position_break_freq_);

        // Task Gains
        myUtils::readParameter(node, "foot_rz_xyz_kp", kp_foot);
        myUtils::readParameter(node, "foot_rz_xyz_kd", kd_foot);

        myUtils::readParameter(node, "com_kp", com_kp);
        myUtils::readParameter(node, "com_kd", com_kd);

        myUtils::readParameter(node, "body_kp", kp_body_rpy);
        myUtils::readParameter(node, "body_kd", kd_body_rpy);

        myUtils::readParameter(node, "joint_kp", kp_jp);
        myUtils::readParameter(node, "joint_kd", kd_jp);

        myUtils::readParameter(node, "w_contact_weight", w_contact_weight_);
        myUtils::readParameter(node, "ihwbc_alpha_fd", alpha_fd_);
       

    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }

    // Set Task Gains
    rfoot_center_rz_xyz_task->setGain(kp_foot, kd_foot);
    lfoot_center_rz_xyz_task->setGain(kp_foot, kd_foot);


    Eigen::VectorXd kp_point_foot = kp_foot.head(3); 
    Eigen::VectorXd kd_point_foot = kd_foot.head(3);

    rfoot_front_task->setGain(kp_point_foot, kd_point_foot);
    rfoot_back_task->setGain(kp_point_foot, kd_point_foot);
    lfoot_front_task->setGain(kp_point_foot, kd_point_foot);
    lfoot_back_task->setGain(kp_point_foot, kd_point_foot);

    Eigen::VectorXd kp_line_task = kp_foot[0]*Eigen::VectorXd::Ones(5);
    Eigen::VectorXd kd_line_task = kd_foot[0]*Eigen::VectorXd::Ones(5);
    kp_line_task[4] = kp_foot[2];
    kd_line_task[4] = kd_foot[2];
    
    rfoot_line_task->setGain(kp_line_task, kd_line_task);
    lfoot_line_task->setGain(kp_line_task, kd_line_task);



    com_task_->setGain(com_kp, com_kd);
    total_joint_task_->setGain(kp_jp, kd_jp);
    body_ori_task_->setGain(kp_body_rpy, kd_body_rpy);

    ang_momentum_task->setGain(Eigen::Vector3d::Zero(), kd_body_rpy);

    // Set IHWBC dt integration time
    ihwbc_dt_ = DracoAux::ServoRate; 
}
