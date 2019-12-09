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

MPCBalanceCtrl::MPCBalanceCtrl(RobotSystem* robot) : Controller(robot) {
    myUtils::pretty_constructor(2, "MPC Balance Ctrl");

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

    // contact
    rfoot_front_contact_ = new PointContactSpec(robot_, DracoBodyNode::rFootFront, 0.9);
    rfoot_back_contact_ = new PointContactSpec(robot_, DracoBodyNode::rFootBack, 0.9);
    lfoot_front_contact_ = new PointContactSpec(robot_, DracoBodyNode::lFootFront, 0.9);
    lfoot_back_contact_ = new PointContactSpec(robot_, DracoBodyNode::lFootBack, 0.9);

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
    mpc_mu_ = 0.9; // Coefficient of Friction on each contact point
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

    // ICP
    kp_ic_ = 20.0; // ICP P gain
    ki_ic_ = 1.0;  // ICP I gain
    icp_sat_error_ = 1e-1; //1e-2;
    icp_acc_error_ = Eigen::VectorXd::Zero(2);


    // wbc
    std::vector<bool> act_list;
    act_list.resize(robot_->getNumDofs(), true);
    for (int i(0); i < robot_->getNumVirtualDofs(); ++i) act_list[i] = false;


    // IHWBC
    last_control_time_ = -0.001;
    ihwbc = new IHWBC(act_list);
    gamma_old_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

    // Regularization terms should always be the lowest cost. 
    lambda_qddot_ = 1e-14;
    lambda_Fr_ = 1e-16;

    // Relative task weighting
    w_task_rfoot_ = 1.0;
    w_task_lfoot_ = 1.0;
    w_task_com_ = 1e-4;
    w_task_body_ = 1e-4;
    w_task_joint_ = 1e-6;

    // Relative reaction force tracking weight compared to tasks
    // Must be high if desired reaction force is nonzero. 
    w_contact_weight_ = 1e-2; 

    // Initialize State Provider
    sp_ = DracoStateProvider::getStateProvider(robot_);
}

MPCBalanceCtrl::~MPCBalanceCtrl() {
    delete com_task_;
    delete total_joint_task_;

    delete rfoot_front_contact_;
    delete rfoot_back_contact_;
    delete lfoot_front_contact_;
    delete lfoot_back_contact_;
}

double MPCBalanceCtrl::clamp_value(double in, double min, double max){
    if (in >= max){
        return max;
    }else if (in <= min){
        return min;
    }else{
        return in;
    }
}

void MPCBalanceCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;
    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

    // Setup the contacts
    contact_setup();

    // Run the MPC at every MPC tick
    if (((state_machine_time_ - last_control_time_) > mpc_dt_) || (last_control_time_ < 0)){
        last_control_time_ = state_machine_time_;
        // // Setup and solve the MPC 
        _mpc_setup();
        _mpc_Xdes_setup();
        // _mpc_solve();
    }
    // Setup the tasks and compute torque from IHWBC
    task_setup();
    // clock_.start();
    _compute_torque_ihwbc(gamma);
    // printf("time: %f\n", clock_.stop());
    // Store the desired feed forward torque command 
    gamma_old_ = gamma;

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

void MPCBalanceCtrl::_mpc_setup(){
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
    // myUtils::pretty_print(mpc_r_feet_, std::cout, "mpc_r_feet_");
    q_current_ = sp_->q; //robot_->getQ();
    qdot_current_ = sp_->qdot; //robot_->getQdot();

    com_current_ = sp_->com_pos; //robot_->getCoMPosition();
    com_rate_current_ = sp_->est_com_vel; //robot_->getCoMVelocity();

    // myUtils::pretty_print(com_current_, std::cout, "com_current_");

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

    // myUtils::pretty_print(midfeet_pos_, std::cout, "midfeet_pos_");
    // myUtils::pretty_print(q_current_, std::cout, "q_current_");
    // myUtils::pretty_print(qdot_current_, std::cout, "qdot_current_");
    // myUtils::pretty_print(mpc_x0_, std::cout, "mpc_x0_");


}

void MPCBalanceCtrl::_mpc_Xdes_setup(){
    int n = convex_mpc->getStateVecDim(); // This is always size 13.
    mpc_Xdes_ = Eigen::VectorXd::Zero(n*mpc_horizon_); // Create the desired state vector evolution

    double magnitude = sway_magnitude_;
    double T = sway_period_;
    double freq = 1/T;
    double omega = 2 * M_PI * freq;

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

void MPCBalanceCtrl::_mpc_solve(){
    // Solve the mpc
    convex_mpc->solve_mpc(mpc_x0_, mpc_Xdes_, mpc_r_feet_, mpc_x_pred_, mpc_Fd_out_);
    mpc_Fd_des_ = convex_mpc->getComputedGroundForces();

    // Update predicted behavior to state estimate
    sp_->mpc_pred_pos = mpc_x_pred_.segment(3,3);
    sp_->mpc_pred_vel = mpc_x_pred_.segment(9,3);

    // myUtils::pretty_print(mpc_x0_, std::cout, "mpc_x0_");
    // myUtils::pretty_print(mpc_x_pred_, std::cout, "mpc_x_pred_");
    // myUtils::pretty_print(mpc_Fd_des_, std::cout, "mpc_Fd_des_");

}

void MPCBalanceCtrl::_compute_torque_ihwbc(Eigen::VectorXd& gamma) {
    // When Fd is nonzero, we need to make the contact weight large if we want to trust the output of the 
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
    des_jvel_ = qdot_des_;
    des_jpos_ = q_des_;

    // des_jpos_ = des_jpos_ + (des_jvel_* ihwbc_dt_) + (0.5 * qddot_cmd_ * ihwbc_dt_ * ihwbc_dt_);
    // des_jvel_ = des_jvel_ + (qddot_cmd_*ihwbc_dt_);


    // Store desired qddot
    sp_->qddot_cmd = qddot_res;    

    // Store desired reaction force data
    sp_->reaction_forces = mpc_Fd_des_;
    sp_->filtered_rf = Fr_res;

    // myUtils::pretty_print(mpc_Fd_des_, std::cout, "mpc_Fd_des_");
    // myUtils::pretty_print(tau_cmd_, std::cout, "tau_cmd_");
    // myUtils::pretty_print(qdot_des_, std::cout, "qdot_des_");
    // myUtils::pretty_print(q_des_, std::cout, "q_des_");

    // myUtils::pretty_print(ac_qdot_current, std::cout, "ac_qdot_current");
    // myUtils::pretty_print(ac_q_current, std::cout, "ac_q_current");
    Eigen::VectorXd com_pos_cur = sp_->com_pos;

    // myUtils::pretty_print(com_pos_cur, std::cout, "com_pos_cur");
    // myUtils::pretty_print(qddot_res, std::cout, "qddot_res");
    // myUtils::pretty_print(qddot_cmd_, std::cout, "qddot_cmd_");
    // myUtils::pretty_print(des_jpos_, std::cout, "des_jpos_");
    // myUtils::pretty_print(des_jvel_, std::cout, "des_jvel_");

    // myUtils::pretty_print(Fr_res, std::cout, "Fr_res");
}

void MPCBalanceCtrl::task_setup() {
    // Set desired com and body orientation from predicted state 
    double des_roll = mpc_Xdes_[0]; //mpc_x_pred_[0];
    double des_pitch = mpc_Xdes_[1]; //mpc_x_pred_[1];
    double des_yaw = mpc_Xdes_[2]; //mpc_x_pred_[2];

    double des_pos_x = mpc_Xdes_[3]; //mpc_x_pred_[3];
    double des_pos_y = mpc_Xdes_[4]; //mpc_x_pred_[4];
    double des_pos_z = mpc_Xdes_[5]; //mpc_x_pred_[5];

    double des_roll_rate = mpc_Xdes_[6]; //mpc_x_pred_[6];
    double des_pitch_rate = mpc_Xdes_[7]; //mpc_x_pred_[7];
    double des_yaw_rate = mpc_Xdes_[8]; //mpc_x_pred_[8];

    double des_vel_x = mpc_Xdes_[9]; //mpc_x_pred_[9];
    double des_vel_y = mpc_Xdes_[10]; //mpc_x_pred_[10];
    double des_vel_z = mpc_Xdes_[11]; //mpc_x_pred_[11];

    // =========================================================================
    // Com Task
    // =========================================================================
    Eigen::VectorXd com_pos_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd com_vel_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd com_acc_des = Eigen::VectorXd::Zero(3);

    if (state_machine_time_ < stab_dur_) {
        for (int i = 0; i < 3; ++i) {
            com_pos_des[i] = myUtils::smooth_changing(ini_com_pos_[i], goal_com_pos_[i], stab_dur_, state_machine_time_);
            com_vel_des[i] = myUtils::smooth_changing_vel(ini_com_vel_[i], 0., stab_dur_, state_machine_time_);
            com_acc_des[i] = 0.;
        }
    } else {
        for (int i = 0; i < 3; ++i) {
            com_pos_des[i] = goal_com_pos_[i];
            com_vel_des[i] = 0.;
            com_acc_des[i] = 0.;
        }
    }

    // Define capture point linear momentum task:
    // Eigen::Vector3d com_pos = sp_->com_pos; //robot_->getCoMPosition();
    // Eigen::Vector3d com_vel = sp_->est_com_vel ;// robot_->getCoMVelocity();

    double gravity = 9.81;
    double com_height = sp_->com_pos[2];
    double omega_o = std::sqrt(gravity/com_height);

    // Current ICP 
    Eigen::VectorXd r_ic = sp_->com_pos.head(2) + (1.0/omega_o)*sp_->est_com_vel.head(2);
    // Desired ICP
    Eigen::VectorXd r_id = goal_com_pos_.head(2);
    // Desired ICP Velocity
    Eigen::VectorXd rdot_id = Eigen::VectorXd::Zero(2);

    // Desired CMP
    Eigen::VectorXd r_icp_error = (r_ic - r_id);
    icp_acc_error_ += (r_icp_error*ihwbc_dt_);
    icp_acc_error_[0] = clamp_value(icp_acc_error_[0], -icp_sat_error_, icp_sat_error_);
    icp_acc_error_[1] = clamp_value(icp_acc_error_[1], -icp_sat_error_, icp_sat_error_);

    Eigen::VectorXd r_CMP_d = r_ic - rdot_id/omega_o + kp_ic_ * (r_icp_error) + ki_ic_*icp_acc_error_;

    // myUtils::pretty_print(r_icp_error, std::cout, "r_icp_error");
    // myUtils::pretty_print(icp_acc_error_, std::cout, "icp_acc_error_");
    // myUtils::pretty_print(r_icp_error, std::cout, "r_icp_error");

    // Desired Linear Momentum / Acceleration task:
    com_acc_des.head(2) = (gravity/com_height)*( sp_->com_pos.head(2) - r_CMP_d);
    com_acc_des[2] = 0.0;

    // Zero out desired com position and velocity except for the height control.
    com_pos_des = robot_->getCoMPosition();
    com_vel_des = robot_->getCoMVelocity();
    com_pos_des[2] = des_pos_z;
    com_vel_des[2] = des_vel_z;

    // com_pos_des[0] = des_pos_x;
    // com_pos_des[1] = des_pos_y;
    // com_pos_des[2] = des_pos_z;

    // com_vel_des[0] = des_vel_x;
    // com_vel_des[1] = des_vel_y;
    // com_vel_des[2] = des_vel_z;


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

    body_ori_vel_des = myUtils::EulerZYXRatestoAngVel(des_roll, des_pitch, des_yaw,
                                                      des_roll_rate, des_pitch_rate, des_yaw_rate);
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
    task_list_.push_back(total_joint_task_);

    w_task_heirarchy_ = Eigen::VectorXd::Zero(task_list_.size());

    w_task_heirarchy_[0] = w_task_com_; // rfoot
    w_task_heirarchy_[1] = w_task_body_; // rfoot
    w_task_heirarchy_[2] = w_task_rfoot_; // rfoot
    w_task_heirarchy_[3] = w_task_rfoot_; // lfoot
    w_task_heirarchy_[4] = w_task_lfoot_; // rfoot
    w_task_heirarchy_[5] = w_task_lfoot_; // lfoot
    w_task_heirarchy_[6] = w_task_joint_; // joint    

    // w_task_heirarchy_[0] = w_task_com_; // COM
    // w_task_heirarchy_[1] = w_task_body_; // Body Ori

    // w_task_heirarchy_[3] = w_task_rfoot_; // rfoot
    // w_task_heirarchy_[4] = w_task_lfoot_; // lfoot

    // w_task_heirarchy_[3] = w_task_rfoot_; // rfoot
    // w_task_heirarchy_[4] = w_task_rfoot_; // lfoot
    // w_task_heirarchy_[5] = w_task_lfoot_; // rfoot
    // w_task_heirarchy_[6] = w_task_lfoot_; // lfoot

    // w_task_heirarchy_[0] = w_task_rfoot_; // rfoot
    // w_task_heirarchy_[1] = w_task_lfoot_; // lfoot
    // w_task_heirarchy_[2] = w_task_joint_; // joint    

}

void MPCBalanceCtrl::contact_setup() {
    rfoot_front_contact_->updateContactSpec();
    rfoot_back_contact_->updateContactSpec();
    lfoot_front_contact_->updateContactSpec();
    lfoot_back_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_front_contact_);
    contact_list_.push_back(rfoot_back_contact_);
    contact_list_.push_back(lfoot_front_contact_);
    contact_list_.push_back(lfoot_back_contact_);

}

void MPCBalanceCtrl::firstVisit() {
    icp_acc_error_ = Eigen::VectorXd::Zero(2);
    jpos_ini_ = sp_->q.segment(robot_->getNumVirtualDofs(),
                               robot_->getNumActuatedDofs());
    jdot_ini_ = sp_->qdot.segment(robot_->getNumVirtualDofs(),
                                  robot_->getNumActuatedDofs());

    des_jpos_ = jpos_ini_;
    des_jvel_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

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

    std::cout << "ICP kp: " << kp_ic_ << std::endl;
    std::cout << "ICP ki: " << ki_ic_ << std::endl;
    std::cout << "ICP saturation error: " << icp_sat_error_ << std::endl;

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

void MPCBalanceCtrl::lastVisit() {}

bool MPCBalanceCtrl::endOfPhase() {
    if (state_machine_time_ > end_time_) {
        return true;
    }
    return false;
}

void MPCBalanceCtrl::ctrlInitialization(const YAML::Node& node) {
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
        // ICP Control Parameters
        myUtils::readParameter(node, "kp_icp", kp_ic_);
        myUtils::readParameter(node, "ki_icp", ki_ic_);
        myUtils::readParameter(node, "icp_sat_error", icp_sat_error_);


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
