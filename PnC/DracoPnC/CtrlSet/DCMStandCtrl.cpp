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

DCMStandCtrl::DCMStandCtrl(RobotSystem* robot) : Controller(robot) {
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

DCMStandCtrl::~DCMStandCtrl() {
    delete com_task_;
    delete total_joint_task_;

    delete left_foot_start_;
    delete right_foot_start_;

    delete rfoot_front_contact_;
    delete rfoot_back_contact_;
    delete lfoot_front_contact_;
    delete lfoot_back_contact_;
}

double DCMStandCtrl::clamp_value(double in, double min, double max){
    if (in >= max){
        return max;
    }else if (in <= min){
        return min;
    }else{
        return in;
    }
}

double DCMStandCtrl::computeAlphaGivenBreakFrequency(double hz, double dt){
    double omega = 2.0 * M_PI * hz;
    double alpha = (1.0 - (omega*dt/2.0)) / (1.0 + (omega*dt/2.0));
    alpha = clamp_value(alpha, 0.0, 1.0);
    return alpha;
}

void DCMStandCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;
    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

    // Get trajectory state
    ctrl_state_ =  DRACO_STATE_DS; 

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
    gamma_old_ = gamma*alphaTau + (1.0 - alphaTau)*gamma_old_;

    // Send the Commands
    for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
        ((DracoCommand*)_cmd)->jtrq[i] = gamma_old_[i];
        ((DracoCommand*)_cmd)->q[i] = des_jpos_[i];
        ((DracoCommand*)_cmd)->qdot[i] = des_jvel_[i];
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

void DCMStandCtrl::_Xdes_setup(){
    int n = 13; 
    Xdes_ = Eigen::VectorXd::Zero(n*1); // Create the desired state vector evolution

    double magnitude = 0.0;
    double T = 1.0;
    double freq = 1/T;
    double omega = 2 * M_PI * freq;

    // Angular velocity and acceleration references
    double t_query = 0.0;

    // Time 
    t_query = state_machine_time_;

    Xdes_[0] = 0.0; // Desired Roll
    Xdes_[1] = 0.0; // Desired Pitch
    Xdes_[2] = 0.0; // Desired Yaw

    // ----------------------------------------------------------------
    // Set CoM Position -----------------------------------------------
    Xdes_[3] = goal_com_pos_[0];
    Xdes_[4] = goal_com_pos_[1];

    // Wait for contact transition to finish
    if (t_query <= contact_transition_dur_){
        Xdes_[5] = ini_com_pos_[2];
    }else {
        Xdes_[5] = myUtils::smooth_changing(ini_com_pos_[2], target_com_height_, stab_dur_, (t_query - contact_transition_dur_) ); // Desired com z
    }

    // Set CoM Velocity -----------------------------------------------
    Xdes_[9] = 0.0;
    Xdes_[10] = 0.0;

    // Wait for contact transition to finish
    if (t_query <= contact_transition_dur_){
        Xdes_[11] = 0.0;
    }else {
        Xdes_[11] = myUtils::smooth_changing_vel(ini_com_vel_[2], 0., stab_dur_, (t_query - contact_transition_dur_)); // Desired com z
    }

    for (int i = 0; i < 3; ++i) {
        sp_->com_pos_des[i] = Xdes_[i+3];
        sp_->com_vel_des[i] = Xdes_[i+9];
    }

}

void DCMStandCtrl::_compute_torque_ihwbc(Eigen::VectorXd& gamma) {
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
    sp_->filtered_rf = Fd_des_;
    sp_->reaction_forces = Fr_res;

}

void DCMStandCtrl::task_setup() {
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


    if (state_machine_time_ < (stab_dur_ + contact_transition_dur_)){
        des_jpos_ = sp_->q.segment(robot_->getNumVirtualDofs(),
                                   robot_->getNumActuatedDofs());
        des_jvel_ = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());   
    }

    Eigen::Vector3d com_acc_des, com_vel_des, com_pos_des;

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

    Eigen::VectorXd foot_pos_d(3); foot_pos_d.setZero();
    Eigen::VectorXd foot_vel_d(3); foot_vel_d.setZero();
    Eigen::VectorXd foot_acc_d(3); foot_acc_d.setZero();

    // Compute swing foot trajectories --------------------------------------------------------------
    Eigen::Vector3d f_pos, f_vel, f_acc;
    Eigen::Quaterniond f_ori;       
    Eigen::Vector3d f_ori_vel, f_ori_acc;
    Eigen::Vector3d toe_pos;
    Eigen::Vector3d heel_pos;

    // Set Point Tasks
    foot_pos_d =  robot_->getBodyNodeCoMIsometry(DracoBodyNode::rFootFront).translation();   
    rfoot_front_task->updateTask(foot_pos_d, foot_vel_d, foot_acc_d);
    foot_pos_d = robot_->getBodyNodeCoMIsometry(DracoBodyNode::rFootBack).translation();
    rfoot_back_task->updateTask(foot_pos_d, foot_vel_d, foot_acc_d);        
    // Set Line Task
    rfoot_line_task->updateTask(rfoot_pos_des, foot_vel_des, foot_acc_des);

    // Set Point Tasks
    foot_pos_d = robot_->getBodyNodeCoMIsometry(DracoBodyNode::lFootFront).translation();
    lfoot_front_task->updateTask(foot_pos_d, foot_vel_d, foot_acc_d);
    foot_pos_d = robot_->getBodyNodeCoMIsometry(DracoBodyNode::lFootBack).translation();
    lfoot_back_task->updateTask(foot_pos_d, foot_vel_d, foot_acc_d);        
    // Set Line Task
    lfoot_line_task->updateTask(lfoot_pos_des, foot_vel_des, foot_acc_des);

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


void DCMStandCtrl::contact_setup() {
    rfoot_front_contact_->updateContactSpec();
    rfoot_back_contact_->updateContactSpec();
    lfoot_front_contact_->updateContactSpec();
    lfoot_back_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_front_contact_);
    contact_list_.push_back(rfoot_back_contact_);
    contact_list_.push_back(lfoot_front_contact_);
    contact_list_.push_back(lfoot_back_contact_);

    // Smoothly change maximum Fz for IHWBC
    double smooth_max_fz = myUtils::smooth_changing(0.0, max_fz_, contact_transition_dur_, state_machine_time_ );
 
    for(int i = 0; i < contact_list_.size(); i++){
        ((PointContactSpec*)contact_list_[i])->setMaxFz(smooth_max_fz);            
    }   
 }

void DCMStandCtrl::firstVisit() {
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
    goal_com_pos_[2] = target_com_height_;


    std::cout << "Integration Params" << std::endl;
    std::cout << "  Max Joint Velocity: " << max_joint_vel_ << std::endl;
    std::cout << "  Velocity Break Freq : " << velocity_break_freq_ << std::endl;
    std::cout << "  Max Joint Position Error : " << max_jpos_error_ << std::endl;
    std::cout << "  Position Break Freq : " << position_break_freq_ << std::endl;

    std::cout << "IHWBC reaction force alpha:" << alpha_fd_ << std::endl;

}

void DCMStandCtrl::lastVisit() {}

bool DCMStandCtrl::endOfPhase() {
    if (state_machine_time_ > end_time_) {
        return true;
    }
    return false;
}

void DCMStandCtrl::ctrlInitialization(const YAML::Node& node) {
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
