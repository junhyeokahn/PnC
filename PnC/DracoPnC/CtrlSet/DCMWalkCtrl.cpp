#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/WBC/IHWBC/IHWBC.hpp>
#include <PnC/MPC/CMPC.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/Math/MathUtilities.hpp>
#include <PnC/MPC/MPCDesiredTrajectoryManager.hpp>
#include <PnC/GaitCycle/GaitCycle.hpp>

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

    des_jpos_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
    des_jvel_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());
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
    convex_mpc = new CMPC();
    mpc_horizon_ = 20; // steps
    mpc_dt_ = 0.025; // seconds per step
    mpc_mu_ = 0.7; // Coefficient of Friction on each contact point
    mpc_max_fz_ = 1500.0; // Maximum Reaction force on each contact point
    mpc_control_alpha_ = 1e-12; // Regularization term on the reaction force
    mpc_delta_smooth_ = 1e-12; // Smoothing parameter on the reaction force solutions
    mpc_smooth_from_prev_ = false; // Whether to use the previous solution to smooth the current solution

    mpc_toe_heel_smooth_ = 1e-4; // Smoothing parameter between the toe and the heel
    mpc_do_toe_heel_smoothing_ = false; // If custom smoothing should be done

    mpc_Fd_des_ = Eigen::VectorXd::Zero(dim_contact_);
    mpc_Fd_des_filtered_ = Eigen::VectorXd::Zero(dim_contact_);
    alpha_fd_ = 0.9;

    mpc_cost_vec_ = Eigen::VectorXd::Zero(13);
    mpc_cost_vec_ << 2.5, 2.5, 2.5, 30.0, 10.0, 10.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0;        
    mpc_cost_vec_walk_ = mpc_cost_vec_;
    mpc_terminal_cost_vec_ = mpc_cost_vec_;

    mpc_t_start_solve_ = 0.0;
    mpc_solved_once_ = false; // Whether the MPC has been solved at least once

    simulate_mpc_solved_ = false;


    // Footsteps and Reference Trajectory Module
    // Set which contact points from the contact_list_ corresponds to which footstep
    std::vector<int> contact_index_to_side = {DRACO_RIGHT_FOOTSTEP, DRACO_RIGHT_FOOTSTEP,
                                              DRACO_LEFT_FOOTSTEP, DRACO_LEFT_FOOTSTEP};
    reference_trajectory_module_ = new DCMWalkingReferenceTrajectoryModule(contact_index_to_side);

    references_set_once_ = false;

    left_foot_start_ = new DracoFootstep();
    right_foot_start_ = new DracoFootstep();

    // Gait
    no_gait_ = std::make_shared<GaitCycle>();
    // Set custom gait cycle
    gait_swing_time_ = 0.3;
    gait_transition_time_ = 0.2;
    gait_biped_walking_offset_ = gait_swing_time_ + gait_transition_time_;
    gait_total_gait_duration_ = 2.0*gait_swing_time_ + 2.0*gait_transition_time_;
    biped_gait_ = std::shared_ptr<GaitCycle>(new GaitCycle(gait_swing_time_, gait_total_gait_duration_, {0.0, 0.0, gait_biped_walking_offset_, gait_biped_walking_offset_}));
    gait_set_once_ = false;

    // Trajectory managers
    int mpc_state_dim = 13;
    int mpc_contact_dim = 12;
    mpc_old_trajectory_ = new MPCDesiredTrajectoryManager(mpc_contact_dim, mpc_state_dim, mpc_horizon_, mpc_dt_);
    mpc_new_trajectory_ = new MPCDesiredTrajectoryManager(mpc_contact_dim, mpc_state_dim, mpc_horizon_, mpc_dt_);
    homotopy_merge_time_ = mpc_dt_/2.0; // initialize a merge time

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

    delete mpc_old_trajectory_;
    delete mpc_new_trajectory_;

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

    // To Do: Create separate state machines for this....
    // Get trajectory state
    ctrl_state_ =  reference_trajectory_module_->getState(state_machine_time_);
    // printf("t:%0.3f, prev_state: %i, cur_state: %i \n", state_machine_time_, prev_ctrl_state_, ctrl_state_);

    // Setup the contacts
    contact_setup();

    // Setup the Walking Reference Module
    references_setup();

    _mpc_Xdes_setup();

    // Setup the tasks and compute torque from IHWBC
    task_setup();
    // clock_.start();
    _compute_torque_ihwbc(gamma);
    // printf("time: %f\n", clock_.stop());
    // Store the desired feed forward torque command 


    double alphaTau = (1.0 - computeAlphaGivenBreakFrequency(100.0, ihwbc_dt_));   
    gamma_old_ = gamma*alphaTau + (1.0 - alphaTau)*gamma_old_;

    // Send the Commands
    for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
        ((DracoCommand*)_cmd)->jtrq[i] = gamma_old_[i];
        ((DracoCommand*)_cmd)->q[i] = des_jpos_[i];
        ((DracoCommand*)_cmd)->qdot[i] = des_jvel_[i];
    }

    // Sotre the desired DCM and r_vrp references
    if (state_machine_time_ >= walk_start_time_){
        ((DCMWalkingReferenceTrajectoryModule*)reference_trajectory_module_)->dcm_reference.get_ref_dcm(state_machine_time_, sp_->dcm_des);    
        ((DCMWalkingReferenceTrajectoryModule*)reference_trajectory_module_)->dcm_reference.get_ref_dcm_vel(state_machine_time_, sp_->dcm_vel_des);    
        ((DCMWalkingReferenceTrajectoryModule*)reference_trajectory_module_)->dcm_reference.get_ref_r_vrp(state_machine_time_, sp_->r_vrp_des);    
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


    // myUtils::pretty_print(gamma_old_, std::cout, "gamma_old_");
    // myUtils::pretty_print(gamma, std::cout, "gamma");
    // myUtils::pretty_print(des_jpos_, std::cout, "des_jpos_");
    // myUtils::pretty_print(des_jvel_, std::cout, "des_jvel_");

}

void DCMWalkCtrl::references_setup(){

    if (state_machine_time_ >= walk_start_time_){
        if (!references_set_once_){
            // Prepare containers
            Eigen::Vector3d x_com_start;
            Eigen::Quaterniond x_ori_start;
            double init_roll(sp_->q[5]), init_pitch(sp_->q[4]), init_yaw(sp_->q[3]);

            // Get current CoM
            x_com_start = sp_->com_pos;
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

            // Set desired footstep landing locations
            Eigen::Vector3d foot_translate(0.05, 0.0, 0.0);
            Eigen::Quaterniond foot_rotate( Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) );

            // Eigen::Vector3d foot_translate(-0.075, 0.0, 0.0);
            // Eigen::Quaterniond foot_rotate( Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) );

            // Eigen::Vector3d foot_translate(0.0, -0.1, 0.0);
            // Eigen::Quaterniond foot_rotate( Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) );

            // Eigen::Vector3d foot_translate(0.0, 0.0, 0.0);
            // Eigen::Quaterniond foot_rotate( Eigen::AngleAxisd(-M_PI/8.0, Eigen::Vector3d::UnitZ()) );

            DracoFootstep rfootstep_1; // take a rightfootstep
            rfootstep_1.setPosOriSide(foot_rotate.toRotationMatrix()*(right_foot_start_->position) + foot_translate, 
                                      foot_rotate*right_foot_start_->orientation, 
                                      DRACO_RIGHT_FOOTSTEP);

            DracoFootstep lfootstep_1; // take a left footstep
            lfootstep_1.setPosOriSide(foot_rotate.toRotationMatrix()*(left_foot_start_->position) + foot_translate, 
                                      foot_rotate*left_foot_start_->orientation, 
                                      DRACO_LEFT_FOOTSTEP);

            double double_contact_time_in = 0.05;
            double contact_transition_time_in = 0.2;
            double swing_time_in = 0.3;
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
            rfootstep_2.setPosOriSide(rfootstep_1.position + foot_translate, 
                                      foot_rotate*rfootstep_1.orientation, 
                                      DRACO_RIGHT_FOOTSTEP);

            DracoFootstep lfootstep_2; // take a leftfootstep
            lfootstep_2.setPosOriSide(lfootstep_1.position + foot_translate, 
                                      foot_rotate*lfootstep_1.orientation, 
                                      DRACO_LEFT_FOOTSTEP);


            // Clear then add footsteps to the list.
            desired_footstep_list_.clear();
            desired_footstep_list_.push_back(rfootstep_1);
            desired_footstep_list_.push_back(lfootstep_1);
            // desired_footstep_list_.push_back(rfootstep_2);
            // desired_footstep_list_.push_back(lfootstep_2);

            // desired_footstep_list_.push_back(lfootstep_1);
            // desired_footstep_list_.push_back(rfootstep_1);
            // desired_footstep_list_.push_back(lfootstep_2);
            // desired_footstep_list_.push_back(rfootstep_2);

            for(int i = 0; i < desired_footstep_list_.size(); i++){
                printf("Step %i:\n", i);
                desired_footstep_list_[i].printInfo();
            }

            // Update the reference trajectory module
            ((DCMWalkingReferenceTrajectoryModule*)reference_trajectory_module_)->dcm_reference.setCoMHeight(target_com_height_);



            reference_trajectory_module_->setFootsteps(walk_start_time_, desired_footstep_list_);

            // update MPC costs for walking.
            // convex_mpc->setCostVec(mpc_cost_vec_walk_);
            // convex_mpc->setTerminalCostVec(true, mpc_terminal_cost_vec_);
            references_set_once_ = true;
        }
    }    
}

void DCMWalkCtrl::_mpc_Xdes_setup(){
    int n = convex_mpc->getStateVecDim(); // This is always size 13.
    mpc_Xdes_ = Eigen::VectorXd::Zero(n*mpc_horizon_); // Create the desired state vector evolution

    double magnitude = 0.0;
    double T = 1.0;
    double freq = 1/T;
    double omega = 2 * M_PI * freq;

    // Set custom gait cycle
    // if (state_machine_time_ >= walk_start_time_){
    //     if (!gait_set_once_){
    //         convex_mpc->setCustomGaitCycle(biped_gait_);            
    //         gait_set_once_ = true;
    //     }
    // }

    Eigen::Vector3d com_pos_ref, com_vel_ref;
    Eigen::Quaterniond ori_ref;
    Eigen::Vector3d euler_yaw_pitch_roll;

    // Angular velocity and acceleration references
    Eigen::Vector3d ang_vel_ref, ang_acc_ref;
    ang_vel_ref.setZero(); ang_acc_ref.setZero();

    double t_predict = 0.0;
    // std::cout << "MPC X_des for " << mpc_horizon_ << " horizon steps at " << mpc_dt_ << "seconds each interval" << std::endl;  

    // std::cout << "state time:" << state_machine_time_ << std::endl;
    for(int i = 0; i < mpc_horizon_; i++){
        // Time 
        t_predict = state_machine_time_ + (i+1)*mpc_dt_;
        // reference_trajectory_module_->getMPCRefComAndOri(t_predict, com_pos_ref, ori_ref);

        if (state_machine_time_ >= walk_start_time_){
            reference_trajectory_module_->getMPCRefComPosandVel(t_predict, com_pos_ref, com_vel_ref);
            reference_trajectory_module_->getMPCRefQuatAngVelAngAcc(t_predict, ori_ref, ang_vel_ref, ang_acc_ref);            
        }


        mpc_Xdes_[i*n + 0] = 0.0; // Desired Roll
        mpc_Xdes_[i*n + 1] = 0.0; // Desired Pitch
        mpc_Xdes_[i*n + 2] = 0.0; // Desired Yaw

        if (t_predict >= walk_start_time_){
            euler_yaw_pitch_roll = myUtils::QuatToEulerZYX(ori_ref);
            mpc_Xdes_[i*n + 0] = euler_yaw_pitch_roll[2]; // Desired Roll
            mpc_Xdes_[i*n + 1] = euler_yaw_pitch_roll[1]; // Desired Pitch
            mpc_Xdes_[i*n + 2] = euler_yaw_pitch_roll[0]; // Desired Yaw            
        }


        // ----------------------------------------------------------------
        // Set CoM Position -----------------------------------------------
        mpc_Xdes_[i*n + 3] = goal_com_pos_[0];
        // mpc_Xdes_[i*n + 3] = goal_com_pos_[0] + magnitude*cos(omega * t_predict); 

        mpc_Xdes_[i*n + 4] = goal_com_pos_[1];

        // there's a brief moment where com_pos_ref is 0.0. which causes the robot to lean on the other foot.
        // the fix is probably to use a dcm reference trajectory.
        // if (t_predict >= walk_start_time_){
        if (state_machine_time_ >= walk_start_time_){
            mpc_Xdes_[i*n + 3] = com_pos_ref[0];
            mpc_Xdes_[i*n + 4] = com_pos_ref[1];
        }

        // Wait for contact transition to finish
        if (t_predict <= contact_transition_dur_){
            mpc_Xdes_[i*n + 5] = ini_com_pos_[2];
        }else {
            mpc_Xdes_[i*n + 5] = myUtils::smooth_changing(ini_com_pos_[2], target_com_height_, stab_dur_, (t_predict - contact_transition_dur_) ); // Desired com z
        }

        if (state_machine_time_ >= walk_start_time_){
             mpc_Xdes_[i*n + 5] = com_pos_ref[2];
        }
        // ----------------------------------------------------------------

        // Set Angular Velocity ref
        if (state_machine_time_ >= walk_start_time_){
            mpc_Xdes_[i*n + 6] = ang_vel_ref[0];
            mpc_Xdes_[i*n + 7] = ang_vel_ref[1];
            mpc_Xdes_[i*n + 8] = ang_vel_ref[2];            
        }

        // Set CoM Velocity -----------------------------------------------
        mpc_Xdes_[i*n + 9] = 0.0;
        mpc_Xdes_[i*n + 10] = 0.0;

        // Wait for contact transition to finish
        if (t_predict <= contact_transition_dur_){
            mpc_Xdes_[i*n + 11] = 0.0;
        }else {
            mpc_Xdes_[i*n + 11] = myUtils::smooth_changing_vel(ini_com_vel_[2], 0., stab_dur_, (t_predict - contact_transition_dur_)); // Desired com z
        }

        if (state_machine_time_ >= walk_start_time_){
             mpc_Xdes_[i*n + 9] = com_vel_ref[0];
             mpc_Xdes_[i*n + 10] = com_vel_ref[1];
             mpc_Xdes_[i*n + 11] = com_vel_ref[2];
        }

        // TODO: if the swing has ended set desired com position to be the current midfoot.

        // printf("t_pred: %0.3f, r:%0.3f, p:%0.3f, y:%0.3f, x:%0.3f, y:%0.3f, z:%0.3f\n", 
        //         t_predict, mpc_Xdes_[i*n + 0], mpc_Xdes_[i*n + 1], mpc_Xdes_[i*n + 2],
        //         mpc_Xdes_[i*n + 3],  mpc_Xdes_[i*n + 4], mpc_Xdes_[i*n + 5]);

        // std::cout << "t_pred:" << t_predict << " x_des[5] = " << mpc_Xdes_[i*n + 5] << "xdot_des[11] = " << mpc_Xdes_[i*n + 11] << std::endl;
        // std::cout << mpc_Xdes_.segment(i*n, n).transpose() << std::endl;
    }

    // std::cout << "mpc_Xdes_.head(n) = " << mpc_Xdes_.head(n).transpose() << std::endl;

    for (int i = 0; i < 3; ++i) {
        sp_->com_pos_des[i] = mpc_Xdes_[i+3];
        sp_->com_vel_des[i] = mpc_Xdes_[i+9];
    }

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
    int rf_dim = 0;
    for(int i = 0; i < contact_list_.size(); ++i){
        rf_dim += contact_list_[i]->getDim();
    }
    mpc_Fd_des_ = Eigen::VectorXd::Zero(rf_dim);

    ihwbc->solve(task_list_, contact_list_, mpc_Fd_des_filtered_, tau_cmd_, qddot_cmd_);

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
    des_jvel_ = (des_jvel_)*alphaVelocity + (1.0 - alphaVelocity )*qdot_des_ref;
    des_jvel_ += (qddot_cmd_*ihwbc_dt_);
    // Clamp Joint Velocity Values
    for(int i = 0; i < des_jvel_.size(); i++){
        des_jvel_[i] = clamp_value(des_jvel_[i], -max_joint_vel_, max_joint_vel_);
    }
    // myUtils::pretty_print(des_jvel_, std::cout, "des_jvel_");    

    // // Integrate Joint Positions
    Eigen::VectorXd q_des_ref = q_current_.tail(robot_->getNumActuatedDofs());
    double alphaPosition = computeAlphaGivenBreakFrequency(position_break_freq_, ihwbc_dt_);
    des_jpos_ = des_jpos_*alphaPosition + (1.0 - alphaPosition)*q_des_ref;
    des_jpos_ += (des_jvel_*ihwbc_dt_);
    // Clamp desired joint position to maximum error
    for(int i = 0; i < des_jpos_.size(); i++){
        des_jpos_[i] = clamp_value(des_jpos_[i], q_des_ref[i]-max_jpos_error_, q_des_ref[i]+max_jpos_error_);
    }

    // Store desired qddot
    sp_->qddot_cmd = qddot_res;    

    // Store desired reaction force data
    sp_->filtered_rf = mpc_Fd_des_;
    sp_->reaction_forces = Fr_res;

}

void DCMWalkCtrl::task_setup() {
    // Disable MPC
    double des_roll = mpc_Xdes_[0];
    double des_pitch = mpc_Xdes_[1];
    double des_yaw = mpc_Xdes_[2];

    double des_pos_x = mpc_Xdes_[3];
    double des_pos_y = mpc_Xdes_[4];
    double des_pos_z = mpc_Xdes_[5];

    double des_rx_rate = mpc_Xdes_[6];
    double des_ry_rate = mpc_Xdes_[7];
    double des_rz_rate = mpc_Xdes_[8];

    double des_vel_x = mpc_Xdes_[9];
    double des_vel_y = mpc_Xdes_[10];
    double des_vel_z = mpc_Xdes_[11];

    double des_rx_acc = 0.0; 
    double des_ry_acc = 0.0; 
    double des_rz_acc = 0.0; 

    double des_acc_x = 0.0; 
    double des_acc_y = 0.0; 
    double des_acc_z = 0.0; 

    Eigen::Vector3d com_pos_ref, com_vel_ref;
    Eigen::Quaterniond ori_ref;
    Eigen::Vector3d euler_yaw_pitch_roll;

    // Angular velocity and acceleration references
    Eigen::Vector3d ang_vel_ref, ang_acc_ref;
    ang_vel_ref.setZero(); ang_acc_ref.setZero();

    if (state_machine_time_ >= walk_start_time_){
        reference_trajectory_module_->getMPCRefComPosandVel(state_machine_time_, com_pos_ref, com_vel_ref);
        reference_trajectory_module_->getMPCRefQuatAngVelAngAcc(state_machine_time_, ori_ref, ang_vel_ref, ang_acc_ref);            

        euler_yaw_pitch_roll = myUtils::QuatToEulerZYX(ori_ref);
        des_roll = euler_yaw_pitch_roll[2]; // Desired Roll
        des_pitch = euler_yaw_pitch_roll[1]; // Desired Pitch
        des_yaw = euler_yaw_pitch_roll[0]; // Desired Yaw            

        des_pos_x = com_pos_ref[0]; 
        des_pos_y = com_pos_ref[1]; 
        des_pos_z = com_pos_ref[2]; 

        des_rx_rate = ang_vel_ref[0]; 
        des_ry_rate = ang_vel_ref[1]; 
        des_rz_rate = ang_vel_ref[2]; 

        des_vel_x = com_vel_ref[0]; 
        des_vel_y = com_vel_ref[1]; 
        des_vel_z = com_vel_ref[2]; 

        des_rx_acc = 0.0;//ang_acc_ref[0]; 
        des_ry_acc = 0.0;//ang_acc_ref[1]; 
        des_rz_acc = 0.0;//ang_acc_ref[2]; 

        des_acc_x = 0.0; 
        des_acc_y = 0.0; 
        des_acc_z = 0.0;
    }


    Eigen::Vector3d com_acc_des, com_vel_des, com_pos_des;

    if (state_machine_time_ < (stab_dur_ + contact_transition_dur_)){
        des_jpos_ = sp_->q.segment(robot_->getNumVirtualDofs(),
                                   robot_->getNumActuatedDofs());
        des_jvel_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());   
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

    double t_transition = reference_trajectory_module_->reaction_force_schedule_ptr->getTransitionVariable(0, state_machine_time_);
    // std::cout << "time:" << state_machine_time_ << std::endl;
    // std::cout << "index 0: transition: " << t_transition << " max force = " << f_max << std::endl;
 
    // double w_contact_transition = (1-t_transition)*w_task_rfoot_;
    // double w_swing_transition = t_transition*1e-2;

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

    // w_task_heirarchy_[2] = (1-t_transition)*w_task_rfoot_; // rfoot swing
    // w_task_heirarchy_[3] = (1-t_transition)*w_task_lfoot_; // lfoot swing

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
    double smooth_max_fz = myUtils::smooth_changing(0.0, mpc_max_fz_, contact_transition_dur_, state_machine_time_ );
 
    // if (state_machine_time_ >= walk_start_time_){
       // std::cout << "State Machine Time: " << state_machine_time_ << std::endl;
    // }

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
    // Initialize control state as double support
    ctrl_state_ = DRACO_STATE_DS;
    prev_ctrl_state_ = DRACO_STATE_DS;

    jpos_ini_ = sp_->q.segment(robot_->getNumVirtualDofs(),
                               robot_->getNumActuatedDofs());
    jdot_ini_ = sp_->qdot.segment(robot_->getNumVirtualDofs(),
                                  robot_->getNumActuatedDofs());

    des_jpos_ = jpos_ini_;
    des_jvel_ = jdot_ini_;

    ctrl_start_time_ = sp_->curr_time;

    Eigen::Vector3d com_pos = sp_->com_pos; //robot_->getCoMPosition();
    Eigen::Vector3d com_vel = sp_->est_com_vel ;// robot_->getCoMVelocity();
    //Eigen::VectorXd rankle_pos = robot_->getBodyNodeIsometry(DracoBodyNode::rFootCenter).translation();
    //Eigen::VectorXd lankle_pos = robot_->getBodyNodeIsometry(DracoBodyNode::lFootCenter).translation();
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
    goal_com_pos_[0] = 0.0;

    //myUtils::pretty_print(ini_com_pos_, std::cout, "ini_com_pos");
    // exit(0);
    goal_com_pos_[2] = target_com_height_;


    // MPC Initial Setup 
    // System Params
    double robot_mass = robot_->getRobotMass(); //kg
    convex_mpc->setRobotMass(robot_mass); // (kilograms) 

    if (mpc_use_approx_inertia_){
        Eigen::MatrixXd I_body(3,3);
        I_body << mpc_approx_inertia_input_[0], mpc_approx_inertia_input_[1], mpc_approx_inertia_input_[2],
                  mpc_approx_inertia_input_[3], mpc_approx_inertia_input_[4], mpc_approx_inertia_input_[5],
                  mpc_approx_inertia_input_[6], mpc_approx_inertia_input_[7], mpc_approx_inertia_input_[8];         
        // robot_->updateCentroidFrame();
        // Eigen::MatrixXd Ig_o = robot_->getCentroidInertia();
        // I_body = Ig_o.block(0,0,3,3);
        // convex_mpc->setRobotInertia(I_body);
        convex_mpc->setRobotInertia(I_body);
        myUtils::pretty_print(I_body, std::cout, "I_body");
    }

    std::cout << "Integration Params" << std::endl;
    std::cout << "  Max Joint Velocity: " << max_joint_vel_ << std::endl;
    std::cout << "  Velocity Break Freq : " << velocity_break_freq_ << std::endl;
    std::cout << "  Max Joint Position Error : " << max_jpos_error_ << std::endl;
    std::cout << "  Position Break Freq : " << position_break_freq_ << std::endl;

    std::cout << "MPC Robot Weight:" << robot_mass << std::endl;
    std::cout << "MPC Use Approx Inertia:" << mpc_use_approx_inertia_ << std::endl;
    std::cout << "MPC horizon:" << mpc_horizon_ << std::endl;
    std::cout << "MPC dt:" << mpc_dt_ << std::endl;
    std::cout << "MPC control alpha:" << mpc_control_alpha_ << std::endl;
    std::cout << "MPC delta smooth:" << mpc_delta_smooth_ << std::endl;
    std::cout << "MPC Smooth from prev?:" << mpc_smooth_from_prev_ << std::endl;

    std::cout << "MPC Toe Heel Smoothing:" << mpc_do_toe_heel_smoothing_ << std::endl;
    std::cout << "MPC Toe Heel Smoothing Value:" << mpc_toe_heel_smooth_ << std::endl;   

    std::cout << "IHWBC reaction force alpha:" << alpha_fd_ << std::endl;

    std::cout << "walk_start_time:" << walk_start_time_ << std::endl;

    convex_mpc->setHorizon(mpc_horizon_); // horizon timesteps 
    convex_mpc->setDt(mpc_dt_); // (seconds) per horizon
    convex_mpc->setMu(mpc_mu_); //  friction coefficient
    // convex_mpc->setMaxFz(mpc_max_fz_); // (Newtons) maximum vertical reaction force per foot.
    if (mpc_use_approx_inertia_){
        convex_mpc->rotateBodyInertia(true); // False: Assume we are always providing the world inertia
                                              // True: We provide body inertia once        
    }else{
        convex_mpc->rotateBodyInertia(false); 
    }

    convex_mpc->setControlAlpha(mpc_control_alpha_); // Regularization term on the reaction force
    convex_mpc->setSmoothFromPrevResult(mpc_smooth_from_prev_);
    convex_mpc->setDeltaSmooth(mpc_delta_smooth_); // Smoothing parameter on the reaction force results

    // Sets the reaction force schedule
    convex_mpc->setCustomReactionForceSchedule(reference_trajectory_module_->reaction_force_schedule_ptr);

    // Set reference trajectory params
    mpc_old_trajectory_->setHorizon(mpc_horizon_);
    mpc_old_trajectory_->setDt(mpc_dt_);
    mpc_new_trajectory_->setHorizon(mpc_horizon_);
    mpc_new_trajectory_->setDt(mpc_dt_);
    // set merge time of trajectories from mpc
    homotopy_merge_time_ = mpc_dt_/2.0;

    //Penalize forces between the heel and the toe for each leg
    //  toe_heel*||f_{i,toe} - f_{i,heel}|| 
    Eigen::MatrixXd D_toe_heel(6, 12); D_toe_heel.setZero();
    D_toe_heel.block(0,0, 3, 3) = Eigen::MatrixXd::Identity(3,3)*mpc_toe_heel_smooth_;
    D_toe_heel.block(0,3, 3, 3) = -Eigen::MatrixXd::Identity(3,3)*mpc_toe_heel_smooth_;    
    D_toe_heel.block(3,6, 3, 3) = Eigen::MatrixXd::Identity(3,3)*mpc_toe_heel_smooth_;
    D_toe_heel.block(3,9, 3, 3) = -Eigen::MatrixXd::Identity(3,3)*mpc_toe_heel_smooth_;    
    // Create Custom Toe-heel Smoothing Matrix for the horizon:
    Eigen::MatrixXd Dc1(6*((int) mpc_horizon_), 12*((int) mpc_horizon_));
    Dc1.setZero();
    for(int i = 0; i < mpc_horizon_; i++){
        Dc1.block(i*6, i*12, 6, 12) = D_toe_heel;
    }
 
    // Set the custom smoothing
    if (mpc_do_toe_heel_smoothing_){
        convex_mpc->enableCustomSmoothing(true);
        convex_mpc->setCustomSmoothing(Dc1);       
    }

    // Set the cost vector
    convex_mpc->setCostVec(mpc_cost_vec_);

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
       
        myUtils::readParameter(node, "mpc_use_approx_inertia", mpc_use_approx_inertia_);
        if (mpc_use_approx_inertia_){
           myUtils::readParameter(node, "mpc_approx_inertia_input", mpc_approx_inertia_input_);
        }
       
        myUtils::readParameter(node, "mpc_horizon", mpc_horizon_);
        myUtils::readParameter(node, "mpc_dt", mpc_dt_);
        myUtils::readParameter(node, "mpc_cost_vec", mpc_cost_vec_);
        myUtils::readParameter(node, "mpc_cost_vec_walk", mpc_cost_vec_walk_);
        myUtils::readParameter(node, "mpc_terminal_cost_vec", mpc_terminal_cost_vec_);
        myUtils::readParameter(node, "mpc_control_alpha", mpc_control_alpha_);
        myUtils::readParameter(node, "mpc_delta_smooth", mpc_delta_smooth_);
        myUtils::readParameter(node, "mpc_smooth_from_prev", mpc_smooth_from_prev_);

        myUtils::readParameter(node, "mpc_toe_heel_smooth", mpc_toe_heel_smooth_);
        myUtils::readParameter(node, "mpc_do_toe_heel_smoothing", mpc_do_toe_heel_smoothing_);

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
    ihwbc_dt_ = DracoAux::ServoRate; //mpc_dt_;  

}
