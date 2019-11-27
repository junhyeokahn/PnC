#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/WBC/WBLC/KinWBC.hpp>
#include <PnC/WBC/WBLC/WBLC.hpp>
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

    // Convex MPC
    convex_mpc = new CMPC();
    mpc_horizon_ = 10; // steps
    mpc_dt_ = 0.025; // seconds per step

    // wbc
    std::vector<bool> act_list;
    act_list.resize(robot_->getNumDofs(), true);
    for (int i(0); i < robot_->getNumVirtualDofs(); ++i) act_list[i] = false;

    // IHWBC
    ihwbc = new IHWBC(act_list);


    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);

    wblc_data_ = new WBLC_ExtraData();

    wblc_data_->W_qddot_ =
        Eigen::VectorXd::Constant(robot_->getNumDofs(), 100.0);

    wblc_data_->W_rf_ = Eigen::VectorXd::Constant(dim_contact_, 1.0);
    wblc_data_->W_rf_[rfoot_front_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_front_contact_->getDim() + rfoot_back_contact_->getFzIndex()] =
        0.01;
    wblc_data_->W_rf_[rfoot_front_contact_->getDim() + rfoot_back_contact_->getDim() +
        lfoot_front_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_front_contact_->getDim() + rfoot_back_contact_->getDim() +
        lfoot_front_contact_->getDim() + lfoot_back_contact_->getFzIndex()] = 0.01;

    wblc_data_->W_xddot_ = Eigen::VectorXd::Constant(dim_contact_, 1000.0);

    wblc_data_->tau_min_ =
        Eigen::VectorXd::Constant(robot_->getNumActuatedDofs(), -100.);
    wblc_data_->tau_max_ =
        Eigen::VectorXd::Constant(robot_->getNumActuatedDofs(), 100.);

    sp_ = DracoStateProvider::getStateProvider(robot_);
}

MPCBalanceCtrl::~MPCBalanceCtrl() {
    delete com_task_;
    delete total_joint_task_;

    delete kin_wbc_;
    delete wblc_;
    delete wblc_data_;

    delete rfoot_front_contact_;
    delete rfoot_back_contact_;
    delete lfoot_front_contact_;
    delete lfoot_back_contact_;
}

void MPCBalanceCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;
    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(robot_->getNumActuatedDofs());

    contact_setup();
    _mpc_setup();
    _mpc_Xdes_setup();

    task_setup();
    _compute_torque_wblc(gamma);

    for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
        ((DracoCommand*)_cmd)->jtrq[i] = gamma[i];
        ((DracoCommand*)_cmd)->q[i] = des_jpos_[i];
        ((DracoCommand*)_cmd)->qdot[i] = des_jvel_[i];
    }
    _PostProcessing_Command();
}

void MPCBalanceCtrl::_mpc_setup(){
    // Get the initial robot inertia
    robot_->updateCentroidFrame();
    Eigen::MatrixXd Ig_o = robot_->getCentroidInertia();
    Eigen::MatrixXd I_body = Ig_o.block(0,0,3,3);
    convex_mpc->setRobotInertia(I_body);

    // Update Feet Configuration
    // Set Foot contact locations w.r.t world
    Eigen::MatrixXd r_feet(3, contact_list_.size()); // Each column is a reaction force in x,y,z 
    r_feet.setZero();

    int link_id = 0;
    for (int i = 0; i < contact_list_.size(); i++){
        link_id = ((PointContactSpec*)contact_list_[i])->get_link_idx();
        r_feet.col(i) = robot_->getBodyNodeCoMIsometry( link_id ).translation().transpose();        
    }
    // myUtils::pretty_print(r_feet, std::cout, "r_feet");
    Eigen::VectorXd q_current = robot_->getQ();
    Eigen::VectorXd qdot_current = robot_->getQdot();

    Eigen::Vector3d com_current = robot_->getCoMPosition();
    Eigen::Vector3d com_rate_current = robot_->getCoMVelocity();

    // Starting robot state
    // Current reduced state of the robot
    // x = [Theta, p, omega, pdot, g] \in \mathbf{R}^13
    Eigen::VectorXd x0(13); 
    double init_roll(q_current[3]), init_pitch(q_current[4]), init_yaw(q_current[5]), 
         init_com_x(com_current[0]), init_com_y(com_current[1]), init_com_z(com_current[2]), 
         init_roll_rate(qdot_current[3]), init_pitch_rate(qdot_current[4]), init_yaw_rate(qdot_current[5]),
         init_com_x_rate(com_rate_current[0]), init_com_y_rate(com_rate_current[1]), init_com_z_rate(com_rate_current[2]);

    x0 = convex_mpc->getx0(init_roll, init_pitch, init_yaw,
                           init_com_x, init_com_y, init_com_z,
                           init_roll_rate, init_pitch_rate, init_yaw_rate,
                           init_com_x_rate, init_com_y_rate, init_com_z_rate);

    // Set the desired system evolution
}

void MPCBalanceCtrl::_mpc_Xdes_setup(){
    int n = convex_mpc->getStateVecDim(); // This is always size 13.
    mpc_Xdes_ = Eigen::VectorXd::Zero(n*mpc_horizon_); // Create the desired state vector evolution

    double t_predict = 0.0;
    // std::cout << "MPC X_des for " << mpc_horizon_ << " horizon steps at " << mpc_dt_ << "seconds each interval" << std::endl;  
    for(int i = 0; i < mpc_horizon_; i++){
        // Time 
        t_predict = state_machine_time_ + (i+1)*mpc_dt_;

        mpc_Xdes_[i*n + 0] = 0.0; // Desired Roll
        mpc_Xdes_[i*n + 1] = 0.0; // Desired Pitch
        mpc_Xdes_[i*n + 2] = 0.0; // Desired Yaw

        // Set CoM Position
        mpc_Xdes_[i*n + 3] = myUtils::smooth_changing(ini_com_pos_[0], goal_com_pos_[0], stab_dur_, t_predict); // Desired com x
        mpc_Xdes_[i*n + 4] = myUtils::smooth_changing(ini_com_pos_[1], goal_com_pos_[1], stab_dur_, t_predict); // Desired com y
        mpc_Xdes_[i*n + 5] = myUtils::smooth_changing(ini_com_pos_[2], goal_com_pos_[2], stab_dur_, t_predict); // Desired com z

        // Set CoM Velocity
        mpc_Xdes_[i*n + 9] = myUtils::smooth_changing_vel(ini_com_vel_[0], 0., stab_dur_, t_predict); // Desired com x vel
        mpc_Xdes_[i*n + 10] = myUtils::smooth_changing_vel(ini_com_vel_[1], 0., stab_dur_, t_predict); // Desired com y
        mpc_Xdes_[i*n + 11] = myUtils::smooth_changing_vel(ini_com_vel_[2], 0., stab_dur_, t_predict); // Desired com z

        // std::cout << mpc_Xdes_.segment(i*n, n).transpose() << std::endl;
    }
  
  


}

void MPCBalanceCtrl::_mpc_solve(){
    // Solve the mpc
    // convex_mpc.solve_mpc(x0, X_des, r_feet, x_pred, f_vec_out);
}

void MPCBalanceCtrl::_compute_torque_wblc(Eigen::VectorXd& gamma) {
    // WBLC
    Eigen::MatrixXd A_rotor = A_;
    for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
        A_rotor(i + robot_->getNumVirtualDofs(),
                i + robot_->getNumVirtualDofs()) += sp_->rotor_inertia[i];
    }
    Eigen::MatrixXd A_rotor_inv = A_rotor.inverse();

    wblc_->updateSetting(A_rotor, A_rotor_inv, coriolis_, grav_);
    Eigen::VectorXd des_jacc_cmd =
        des_jacc_ +
        Kp_.cwiseProduct(des_jpos_ -
                         sp_->q.segment(robot_->getNumVirtualDofs(),
                                        robot_->getNumActuatedDofs())) +
        Kd_.cwiseProduct(des_jvel_ -
                         sp_->qdot.tail(robot_->getNumActuatedDofs()));

    wblc_->makeWBLC_Torque(des_jacc_cmd, contact_list_, gamma, wblc_data_);

    // myUtils::pretty_print(des_jacc_, std::cout, "des_jacc");
    // myUtils::pretty_print(des_jacc_cmd, std::cout, "des_jacc_cmd");

    sp_->qddot_cmd = wblc_data_->qddot_;
    //for (int i = 0; i < wblc_data_->Fr_.size(); ++i) {
        //sp_->reaction_forces[i] = wblc_data_->Fr_[i];
    //}
    // Eigen::VectorXd Fr_out = wblc_data_->Fr_;
    // myUtils::pretty_print(Fr_out, std::cout, "Fr_out");    
}

void MPCBalanceCtrl::task_setup() {

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
    for (int i = 0; i < 3; ++i) {
    sp_->com_pos_des[i] = com_pos_des[i];
    sp_->com_vel_des[i] = com_vel_des[i];
    }

    com_task_->updateTask(com_pos_des, com_vel_des, com_acc_des);

    // =========================================================================
    // Body Ori Task
    // =========================================================================


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
    task_list_.push_back(total_joint_task_);

    // =========================================================================
    // Solve Inv Kinematics
    // =========================================================================
    kin_wbc_->FindConfiguration(sp_->q, task_list_, contact_list_, des_jpos_,
                                des_jvel_, des_jacc_);
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
    jpos_ini_ = sp_->q.segment(robot_->getNumVirtualDofs(),
                               robot_->getNumActuatedDofs());
    ctrl_start_time_ = sp_->curr_time;

    Eigen::Vector3d com_pos = robot_->getCoMPosition();
    Eigen::Vector3d com_vel = robot_->getCoMVelocity();
    //Eigen::VectorXd rankle_pos = robot_->getBodyNodeIsometry(DracoBodyNode::rFootCenter).translation();
    //Eigen::VectorXd lankle_pos = robot_->getBodyNodeIsometry(DracoBodyNode::lFootCenter).translation();
    Eigen::VectorXd rankle_pos = robot_->getBodyNodeIsometry(DracoBodyNode::rAnkle).translation();
    Eigen::VectorXd lankle_pos = robot_->getBodyNodeIsometry(DracoBodyNode::lAnkle).translation();
    for (int i = 0; i < 3; ++i) {
        ini_com_pos_[i] = com_pos[i];
        ini_com_vel_[i] = com_vel[i];
        goal_com_pos_[i] = (rankle_pos[i] + lankle_pos[i]) / 2.0;
    }
    //myUtils::pretty_print(ini_com_pos_, std::cout, "ini_com_pos");
    //exit(0);
    goal_com_pos_[0] += des_com_offset_x_;
    goal_com_pos_[2] = target_com_height_;


    // MPC Initial Setup 
    // System Params
    double robot_mass = robot_->getRobotMass(); //kg
    convex_mpc->setRobotMass(robot_mass); // (kilograms) 

    convex_mpc->setHorizon(mpc_horizon_); // horizon timesteps 
    convex_mpc->setDt(mpc_dt_); // (seconds) per horizon
    convex_mpc->setMu(0.7); //  friction coefficient
    convex_mpc->setMaxFz(500); // (Newtons) maximum vertical reaction force per foot.
    convex_mpc->rotateBodyInertia(false); // Assume we are always providing the world inertia

    // Set the cost vector
    Eigen::VectorXd cost_vec(13);
    // Vector cost indexing: <<  th1,  th2,  th3,  px,  py,  pz,   w1,  w2,   w3,   dpx,  dpy,  dpz,  g
    cost_vec << 0.25, 0.25, 10.0, 2.0, 2.0, 50.0, 0.0, 0.0, 0.30, 0.20, 0.2, 0.10, 0.0;
    double cost_factor = 8.0;
    cost_vec *= cost_factor;
    convex_mpc->setCostVec(cost_vec);

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

    Eigen::VectorXd com_kp = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd com_kd = Eigen::VectorXd::Zero(3);
    try {
        myUtils::readParameter(node, "kp", Kp_);
        myUtils::readParameter(node, "kd", Kd_);
        myUtils::readParameter(node, "com_kp", com_kp);
        myUtils::readParameter(node, "com_kd", com_kd);
        com_task_->setGain(com_kp, com_kd);
        myUtils::readParameter(node, "desired_offset_x", des_com_offset_x_);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }

    // Set Task Gains
    Eigen::VectorXd kp_foot = 100*Eigen::VectorXd::Ones(4); 
    Eigen::VectorXd kd_foot = 1.0*Eigen::VectorXd::Ones(4);
    rfoot_center_rz_xyz_task->setGain(kp_foot, kd_foot);
    lfoot_center_rz_xyz_task->setGain(kp_foot, kd_foot);

    Eigen::VectorXd kp_jp = Eigen::VectorXd::Ones(Draco::n_adof); 
    Eigen::VectorXd kd_jp = 0.1*Eigen::VectorXd::Ones(Draco::n_adof);
    total_joint_task_->setGain(kp_jp, kd_jp);

    Eigen::VectorXd kp_body_rpy = 100*Eigen::VectorXd::Ones(3); 
    Eigen::VectorXd kd_body_rpy = 1.0*Eigen::VectorXd::Ones(3);
    body_ori_task_->setGain(kp_body_rpy, kd_body_rpy);

}
