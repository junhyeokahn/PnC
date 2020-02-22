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


MPCStandCtrl::MPCStandCtrl(RobotSystem* robot) : Controller(robot) {
    myUtils::pretty_constructor(2, "MPC Stand Ctrl");

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
    mpc_max_fz_ = 500.0; // Maximum Reaction force on each contact point
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

    mpc_t_start_solve_ = 0.0;
    mpc_solved_once_ = false; // Whether the MPC has been solved at least once

    simulate_mpc_solved_ = false;


    // Footsteps and Reference Trajectory Module
    // Set which contact points from the contact_list_ corresponds to which footstep
    std::vector<int> contact_index_to_side = {DRACO_RIGHT_FOOTSTEP, DRACO_RIGHT_FOOTSTEP,
                                              DRACO_LEFT_FOOTSTEP, DRACO_LEFT_FOOTSTEP};
    reference_trajectory_module_ = new WalkingReferenceTrajectoryModule(contact_index_to_side);
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
    mpc_old_trajectory_ = new MPCDesiredTrajectoryManager(13, mpc_horizon_, mpc_dt_);
    mpc_new_trajectory_ = new MPCDesiredTrajectoryManager(13, mpc_horizon_, mpc_dt_);

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
    w_task_rfoot_ = 100.0;
    w_task_lfoot_ = 100.0;
    w_task_com_ = 1e-4;
    w_task_body_ = 1e-4;
    w_task_joint_ = 1e-6;

    w_task_ang_momentum_ = 1e-5;

    // Relative reaction force tracking weight compared to tasks
    // Must be high if desired reaction force is nonzero. 
    w_contact_weight_ = 1e-2; 

    // Initialize State Provider
    sp_ = DracoStateProvider::getStateProvider(robot_);
}

MPCStandCtrl::~MPCStandCtrl() {
    delete com_task_;
    delete total_joint_task_;

    delete rfoot_front_contact_;
    delete rfoot_back_contact_;
    delete lfoot_front_contact_;
    delete lfoot_back_contact_;
}

double MPCStandCtrl::clamp_value(double in, double min, double max){
    if (in >= max){
        return max;
    }else if (in <= min){
        return min;
    }else{
        return in;
    }
}

double MPCStandCtrl::computeAlphaGivenBreakFrequency(double hz, double dt){
    double omega = 2.0 * M_PI * hz;
    double alpha = (1.0 - (omega*dt/2.0)) / (1.0 + (omega*dt/2.0));
    alpha = clamp_value(alpha, 0.0, 1.0);
    return alpha;
}

void MPCStandCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;
    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

    // Setup the contacts
    contact_setup();

    // Setup the Walking Reference Module
    references_setup();

    // update the gait depending on the swing foot
    // To Do

    // Run the MPC at every MPC tick
    double policy_delay = mpc_dt_;
    if ( (!simulate_mpc_solved_) || (last_control_time_ < 0)){
        // // Setup and solve the MPC 
        _mpc_setup();
        _mpc_Xdes_setup();
        _mpc_solve();
        simulate_mpc_solved_ = true;
    }

    // simulate policy delay
   if (((state_machine_time_ - last_control_time_) > policy_delay) || (last_control_time_ < 0)){
       _updateTrajectories();
        last_control_time_ = state_machine_time_;
        simulate_mpc_solved_ = false;
   }

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
    _PostProcessing_Command();        

    // myUtils::pretty_print(gamma_old_, std::cout, "gamma_old_");
    // myUtils::pretty_print(gamma, std::cout, "gamma");
    // myUtils::pretty_print(des_jpos_, std::cout, "des_jpos_");
    // myUtils::pretty_print(des_jvel_, std::cout, "des_jvel_");

}

void MPCStandCtrl::_mpc_setup(){
    // // Get the initial robot inertia
    if (!mpc_use_approx_inertia_){
        robot_->updateCentroidFrame();
        Eigen::MatrixXd Ig_o = robot_->getCentroidInertia();
        Eigen::MatrixXd I_body = Ig_o.block(0,0,3,3);
        convex_mpc->setRobotInertia(I_body);
        // myUtils::pretty_print(I_body, std::cout, "I_world");       
    }

    double smooth_max_fz = myUtils::smooth_changing(0.0, mpc_max_fz_, contact_transition_dur_, state_machine_time_ );
    convex_mpc->setMaxFz(smooth_max_fz); // (Newtons) maximum vertical reaction force per foot.

    // Update Feet Configuration
    // Set Foot contact locations w.r.t world
    mpc_r_feet_ = Eigen::MatrixXd::Zero(3, contact_list_.size()); // Each column is a reaction force in x,y,z 
    mpc_r_feet_.setZero();

    int link_id = 0;
    for (int i = 0; i < contact_list_.size(); i++){
        link_id = ((PointContactSpec*)contact_list_[i])->get_link_idx();
        mpc_r_feet_.col(i) = robot_->getBodyNodeCoMIsometry( link_id ).translation().transpose();        
    }

    // Update States
    q_current_ = sp_->q;
    qdot_current_ = sp_->qdot;

    com_current_ = sp_->com_pos;
    com_rate_current_ = sp_->est_com_vel;

    // Starting robot state
    // Current reduced state of the robot
    // x = [Theta, p, omega, pdot, g] \in \mathbf{R}^13
    double init_roll(q_current_[5]), init_pitch(q_current_[4]), init_yaw(q_current_[3]), 
         init_com_x(com_current_[0]), init_com_y(com_current_[1]), init_com_z(com_current_[2]), 
         init_roll_rate(qdot_current_[5]), init_pitch_rate(qdot_current_[4]), init_yaw_rate(qdot_current_[3]),
         init_com_x_rate(com_rate_current_[0]), init_com_y_rate(com_rate_current_[1]), init_com_z_rate(com_rate_current_[2]);

    mpc_x0_ = convex_mpc->getx0(init_roll, init_pitch, init_yaw,
                           init_com_x, init_com_y, init_com_z,
                           init_roll_rate, init_pitch_rate, init_yaw_rate,
                           init_com_x_rate, init_com_y_rate, init_com_z_rate);

    midfeet_pos_ = 0.5*(robot_->getBodyNodeCoMIsometry(DracoBodyNode::rFootCenter).translation() + 
                        robot_->getBodyNodeCoMIsometry(DracoBodyNode::lFootCenter).translation());


}

void MPCStandCtrl::references_setup(){

    if (state_machine_time_ >= sway_start_time_){
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






            references_set_once_ = true;
        }
    }    
}

void MPCStandCtrl::_mpc_Xdes_setup(){
    int n = convex_mpc->getStateVecDim(); // This is always size 13.
    mpc_Xdes_ = Eigen::VectorXd::Zero(n*mpc_horizon_); // Create the desired state vector evolution

    double magnitude = sway_magnitude_;
    double T = sway_period_;
    double freq = 1/T;
    double omega = 2 * M_PI * freq;

    // Set custom gait cycle
    // if (state_machine_time_ >= sway_start_time_){
    //     if (!gait_set_once_){
    //         convex_mpc->setCustomGaitCycle(biped_gait_);            
    //         gait_set_once_ = true;
    //     }
    // }

    double t_predict = 0.0;
    // std::cout << "MPC X_des for " << mpc_horizon_ << " horizon steps at " << mpc_dt_ << "seconds each interval" << std::endl;  
    for(int i = 0; i < mpc_horizon_; i++){
        // Time 
        t_predict = state_machine_time_ + (i+1)*mpc_dt_;

        mpc_Xdes_[i*n + 0] = 0.0; // Desired Roll
        mpc_Xdes_[i*n + 1] = 0.0; // Desired Pitch
        mpc_Xdes_[i*n + 2] = 0.0; // Desired Yaw

        // ----------------------------------------------------------------
        // Set CoM Position -----------------------------------------------
        mpc_Xdes_[i*n + 3] = goal_com_pos_[0];
        // mpc_Xdes_[i*n + 3] = goal_com_pos_[0] + magnitude*cos(omega * t_predict); 

        mpc_Xdes_[i*n + 4] = goal_com_pos_[1];
        if (t_predict >= sway_start_time_){
            mpc_Xdes_[i*n + 4] = goal_com_pos_[1] + magnitude*sin(omega * (t_predict-sway_start_time_));             
        }

        // Wait for contact transition to finish
        if (t_predict <= contact_transition_dur_){
            mpc_Xdes_[i*n + 5] = ini_com_pos_[2];
        }else{
            mpc_Xdes_[i*n + 5] = myUtils::smooth_changing(ini_com_pos_[2], target_com_height_, stab_dur_, (t_predict - contact_transition_dur_) ); // Desired com z
            // mpc_Xdes_[i*n + 5] = ini_com_pos_[2] + magnitude*cos(omega * t_predict); 
        }

        // ----------------------------------------------------------------
        // Set CoM Velocity -----------------------------------------------
        mpc_Xdes_[i*n + 9] = 0.0;
        // mpc_Xdes_[i*n + 9] = -omega *magnitude*sin(omega * t_predict); 

        mpc_Xdes_[i*n + 10] = 0.0;
        if (t_predict >= sway_start_time_){
            mpc_Xdes_[i*n + 10] = omega*magnitude*cos(omega * (t_predict-sway_start_time_));             
        }

        // Wait for contact transition to finish
        if (t_predict <= contact_transition_dur_){
            mpc_Xdes_[i*n + 11] = 0.0;
        }
        else{
            mpc_Xdes_[i*n + 11] = myUtils::smooth_changing_vel(ini_com_vel_[2], 0., stab_dur_, (t_predict - contact_transition_dur_)); // Desired com z
            // mpc_Xdes_[i*n + 11] = -omega *magnitude*sin(omega * t_predict); 
        }


        // std::cout << mpc_Xdes_.segment(i*n, n).transpose() << std::endl;
    }

    // std::cout << "mpc_Xdes_.head(n) = " << mpc_Xdes_.head(n).transpose() << std::endl;

    for (int i = 0; i < 3; ++i) {
        sp_->com_pos_des[i] = mpc_Xdes_[i+3];
        sp_->com_vel_des[i] = mpc_Xdes_[i+9];
    }

}

void MPCStandCtrl::_mpc_solve(){
    // Solve the mpc
    mpc_t_start_solve_ = state_machine_time_;
    convex_mpc->setPreviewStartTime(mpc_t_start_solve_);

    convex_mpc->solve_mpc(mpc_x0_, mpc_Xdes_, mpc_r_feet_, mpc_x_pred_, mpc_Fd_out_);
    mpc_Fd_des_ = convex_mpc->getComputedGroundForces();

}

void MPCStandCtrl::_updateTrajectories(){
    // Updates the reference trajectories that the IHWBC will follow given a new plan from the MPC
    if (!mpc_solved_once_){
        mpc_old_trajectory_->setStateKnotPoints(mpc_t_start_solve_,
                                        mpc_x0_,
                                        convex_mpc->getXpredOverHorizon()); 
        // Set that this mpc has been solved at least once
        mpc_solved_once_ = true;
    }else{
        // store the old trajectory
        mpc_old_trajectory_->setStateKnotPoints(mpc_new_trajectory_->getStartTime(),
                                                mpc_new_trajectory_->getXStartVector(),
                                                mpc_new_trajectory_->getXpredVector());         
    }
    mpc_new_trajectory_->setStateKnotPoints(mpc_t_start_solve_,
                                    mpc_x0_,
                                    convex_mpc->getXpredOverHorizon()); 
}

void MPCStandCtrl::_compute_torque_ihwbc(Eigen::VectorXd& gamma) {
    // When Fd is nonzero, we need to make the contact weight large if we want to trust the output of the mpc
    // 1e-2/(robot_->getRobotMass()*9.81);
    double local_w_contact_weight = w_contact_weight_/(robot_->getRobotMass()*9.81);

    // Modify Rotor Inertia
    Eigen::MatrixXd A_rotor = A_;
    for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
        A_rotor(i + robot_->getNumVirtualDofs(),
                i + robot_->getNumVirtualDofs()) += sp_->rotor_inertia[i];
    }
    Eigen::MatrixXd A_rotor_inv = A_rotor.inverse();

    // Enable Torque Limits
    ihwbc->enableTorqueLimits(true);
    double tau_lim = 70; //100.0;    
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

    mpc_Fd_des_filtered_ = alpha_fd_*mpc_Fd_des_ + (1.0-alpha_fd_)*mpc_Fd_des_filtered_;

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

void MPCStandCtrl::task_setup() {
    // Disable MPC
    // double des_roll = mpc_Xdes_[0];
    // double des_pitch = mpc_Xdes_[1];
    // double des_yaw = mpc_Xdes_[2];

    // double des_pos_x = mpc_Xdes_[3];
    // double des_pos_y = mpc_Xdes_[4];
    // double des_pos_z = mpc_Xdes_[5];

    // double des_rx_rate = mpc_Xdes_[6];
    // double des_ry_rate = mpc_Xdes_[7];
    // double des_rz_rate = mpc_Xdes_[8];

    // double des_vel_x = mpc_Xdes_[9];
    // double des_vel_y = mpc_Xdes_[10];
    // double des_vel_z = mpc_Xdes_[11];

    // Enable MPC:
    // Set desired com and body orientation from predicted state 
    Eigen::VectorXd x_traj_old;
    Eigen::VectorXd x_traj_new;
    double merge_time = mpc_dt_/2.0;
    double s_merge = (state_machine_time_ - last_control_time_)/merge_time; ;
    mpc_old_trajectory_->getState(state_machine_time_, x_traj_old);
    mpc_new_trajectory_->getState(state_machine_time_, x_traj_new);
    // clamp s
    if (s_merge >= 1){
        s_merge = 1.0;
    }else if (s_merge <= 0){
        s_merge = 0.0;
    }
    mpc_x_pred_ = s_merge*x_traj_new + (1 - s_merge)*x_traj_old;

    // Get desired accelerations
    Eigen::VectorXd xddot_traj_old = mpc_old_trajectory_->getAcc(state_machine_time_);
    Eigen::VectorXd xddot_traj_new = mpc_new_trajectory_->getAcc(state_machine_time_); 
    Eigen::VectorXd xddot_traj_des = s_merge*xddot_traj_new + (1 - s_merge)*xddot_traj_old;


    // Update the behavior
    sp_->mpc_pred_pos = mpc_x_pred_.segment(3,3);
    sp_->mpc_pred_vel = mpc_x_pred_.segment(9,3);

    double des_roll = mpc_x_pred_[0]; 
    double des_pitch = mpc_x_pred_[1]; 
    double des_yaw = mpc_x_pred_[2]; 

    double des_pos_x = mpc_x_pred_[3]; 
    double des_pos_y = mpc_x_pred_[4]; 
    double des_pos_z = mpc_x_pred_[5]; 

    double des_rx_rate = mpc_x_pred_[6]; 
    double des_ry_rate = mpc_x_pred_[7]; 
    double des_rz_rate = mpc_x_pred_[8]; 

    double des_vel_x = mpc_x_pred_[9]; 
    double des_vel_y = mpc_x_pred_[10]; 
    double des_vel_z = mpc_x_pred_[11]; 

    double des_rx_acc = xddot_traj_des[0]; 
    double des_ry_acc = xddot_traj_des[1]; 
    double des_rz_acc = xddot_traj_des[2]; 

    double des_acc_x = xddot_traj_des[3]; 
    double des_acc_y = xddot_traj_des[4]; 
    double des_acc_z = xddot_traj_des[5]; 


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

    foot_pos_d =  robot_->getBodyNodeCoMIsometry(DracoBodyNode::rFootFront).translation();
    rfoot_front_task->updateTask(foot_pos_d, foot_vel_d, foot_acc_d);
    // myUtils::pretty_print(foot_pos_d, std::cout, "rfoot_front_pos");

    foot_pos_d = robot_->getBodyNodeCoMIsometry(DracoBodyNode::rFootBack).translation();
    rfoot_back_task->updateTask(foot_pos_d, foot_vel_d, foot_acc_d);
    // myUtils::pretty_print(foot_pos_d, std::cout, "rfoot_back_pos");

    foot_pos_d = robot_->getBodyNodeCoMIsometry(DracoBodyNode::lFootFront).translation();
    lfoot_front_task->updateTask(foot_pos_d, foot_vel_d, foot_acc_d);
    // myUtils::pretty_print(foot_pos_d, std::cout, "lfoot_front_pos");

    foot_pos_d = robot_->getBodyNodeCoMIsometry(DracoBodyNode::lFootBack).translation();
    lfoot_back_task->updateTask(foot_pos_d, foot_vel_d, foot_acc_d);
    // myUtils::pretty_print(foot_pos_d, std::cout, "lfoot_back_pos");

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
    task_list_.push_back(com_task_);
    task_list_.push_back(body_ori_task_);
    // task_list_.push_back(rfoot_center_rz_xyz_task);
    // task_list_.push_back(lfoot_center_rz_xyz_task);    

    task_list_.push_back(rfoot_front_task);
    task_list_.push_back(rfoot_back_task);    
    task_list_.push_back(lfoot_front_task);
    task_list_.push_back(lfoot_back_task);    
    // task_list_.push_back(total_joint_task_);
    // task_list_.push_back(ang_momentum_task);

    w_task_heirarchy_ = Eigen::VectorXd::Zero(task_list_.size());

    w_task_heirarchy_[0] = w_task_com_; // COM
    w_task_heirarchy_[1] = w_task_body_; // body ori
    w_task_heirarchy_[2] = w_task_rfoot_; // rfoot
    w_task_heirarchy_[3] = w_task_rfoot_; // rfoot
    w_task_heirarchy_[4] = w_task_lfoot_; // rfoot
    w_task_heirarchy_[5] = w_task_lfoot_; // lfoot
    // w_task_heirarchy_[6] = w_task_ang_momentum_; // angular momentum

    // w_task_heirarchy_[6] = w_task_joint_; // joint    

}

void MPCStandCtrl::contact_setup() {
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
    for(int i = 0; i < contact_list_.size(); i++){
        ((PointContactSpec*)contact_list_[i])->setMaxFz(smooth_max_fz);
    }
    

 }

void MPCStandCtrl::firstVisit() {
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

    std::cout << "sway_start_time:" << sway_start_time_ << std::endl;
    std::cout << "sway_magnitude:" << sway_magnitude_ << std::endl;
    std::cout << "sway_period:" << sway_period_ << std::endl;

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

    // Set reference trajectory params
    mpc_old_trajectory_->setHorizon(mpc_horizon_);
    mpc_old_trajectory_->setDt(mpc_dt_);
    mpc_new_trajectory_->setHorizon(mpc_horizon_);
    mpc_new_trajectory_->setDt(mpc_dt_);

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

void MPCStandCtrl::lastVisit() {}

bool MPCStandCtrl::endOfPhase() {
    if (state_machine_time_ > end_time_) {
        return true;
    }
    return false;
}

void MPCStandCtrl::ctrlInitialization(const YAML::Node& node) {
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

    com_task_->setGain(com_kp, com_kd);
    total_joint_task_->setGain(kp_jp, kd_jp);
    body_ori_task_->setGain(kp_body_rpy, kd_body_rpy);

    // Set IHWBC dt integration time
    ihwbc_dt_ = DracoAux::ServoRate; //mpc_dt_;  

}
