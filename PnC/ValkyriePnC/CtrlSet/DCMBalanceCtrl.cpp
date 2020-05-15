#include <PnC/ValkyriePnC/ContactSet/ContactSet.hpp>
#include <PnC/ValkyriePnC/CtrlSet/CtrlSet.hpp>
#include <PnC/ValkyriePnC/TaskSet/TaskSet.hpp>
#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>
#include <PnC/ValkyriePnC/ValkyrieInterface.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>
#include <PnC/WBC/IHWBC/IHWBC.hpp>
#include <PnC/WBC/IHWBC/IHWBC_JointIntegrator.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/Math/MathUtilities.hpp>

DCMBalanceCtrl::DCMBalanceCtrl(RobotSystem* robot) : Controller(robot) {
    myUtils::pretty_constructor(2, "DCM Balance Ctrl");
    ctrl_start_time_ = 0.;
    end_time_ = 0.;
    des_jpos_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    des_jvel_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    des_jacc_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    Kp_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    Kd_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);

    ini_com_pos_ = Eigen::VectorXd::Zero(3);
    des_com_pos_ = Eigen::VectorXd::Zero(3);
    des_com_vel_ = Eigen::VectorXd::Zero(3);
    des_com_acc_ = Eigen::VectorXd::Zero(3);

    com_pos_dev_ = Eigen::VectorXd::Zero(3);

    ini_pelvis_quat_ = Eigen::Quaternion<double> (1,0,0,0);
    // TASK
    com_task_ = new CoMxyz(robot);
    ang_momentum_task_ = new AngularMomentumTask(robot, ValkyrieAux::servo_rate);
    pelvis_ori_task_ = new BasicTask(robot, BasicTaskType::LINKORI, 3, ValkyrieBodyNode::pelvis);

    // Set Upper Body Joint Tasks
    upper_body_joint_indices_.clear();
    for(int i = ValkyrieDoF::torsoYaw; i < (ValkyrieDoF::rightForearmYaw + 1); i++){
        upper_body_joint_indices_.push_back(i);
    }
    upper_body_task_ = new SelectedJointTasks(robot, upper_body_joint_indices_);

    // Set Foot Motion Tasks
    rfoot_center_pos_task = new BasicTask(robot, BasicTaskType::LINKXYZ, 3, ValkyrieBodyNode::rightCOP_Frame);
    lfoot_center_pos_task = new BasicTask(robot, BasicTaskType::LINKXYZ, 3, ValkyrieBodyNode::leftCOP_Frame);
    rfoot_center_ori_task = new BasicTask(robot, BasicTaskType::LINKORI, 3, ValkyrieBodyNode::rightCOP_Frame);
    lfoot_center_ori_task = new BasicTask(robot, BasicTaskType::LINKORI, 3, ValkyrieBodyNode::leftCOP_Frame);

    // Joint Integrator
    ihwbc_joint_integrator_ = new IHWBC_JointIntegrator(Valkyrie::n_adof, ValkyrieAux::servo_rate);

    // Initialize IHWBC
    std::vector<bool> act_list;
    act_list.resize(Valkyrie::n_dof, true);
    for (int i(0); i < Valkyrie::n_vdof; ++i) act_list[i] = false;

    ihwbc_ = new IHWBC(act_list);

    lambda_qddot_ = 1e-8;
    lambda_Fr_ = 1e-8;

    // Initialize Foot Contacts
    rfoot_contact_ = new SurfaceContactSpec(
        robot_, ValkyrieBodyNode::rightCOP_Frame, 0.135, 0.08, 0.7);
    lfoot_contact_ = new SurfaceContactSpec(
        robot_, ValkyrieBodyNode::leftCOP_Frame, 0.135, 0.08, 0.7);
    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();

    // torque limit default setting
    // wblc_data_->tau_min_ = Eigen::VectorXd::Constant(Valkyrie::n_adof, -2500.);
    // wblc_data_->tau_max_ = Eigen::VectorXd::Constant(Valkyrie::n_adof, 2500.);

    sp_ = ValkyrieStateProvider::getStateProvider(robot);

}

DCMBalanceCtrl::~DCMBalanceCtrl() {
    delete com_task_;
    delete ang_momentum_task_;
    delete pelvis_ori_task_;
    delete upper_body_task_;
    delete rfoot_center_pos_task;
    delete lfoot_center_pos_task;
    delete rfoot_center_ori_task;
    delete lfoot_center_ori_task;
    delete ihwbc_;
    delete rfoot_contact_;
    delete lfoot_contact_;
}

void DCMBalanceCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;

    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    _contact_setup();
    _task_setup();
    _compute_torque_wbc(gamma);

    for (int i(0); i < Valkyrie::n_adof; ++i) {
        ((ValkyrieCommand*)_cmd)->jtrq[i] = gamma[i];
        ((ValkyrieCommand*)_cmd)->q[i] = des_jpos_[i];
        ((ValkyrieCommand*)_cmd)->qdot[i] = des_jvel_[i];
    }

    _PostProcessing_Command();
}

void DCMBalanceCtrl::_compute_torque_wbc(Eigen::VectorXd& gamma) {
    // Set QP weights
    double local_w_contact_weight = w_contact_weight_/(robot_->getRobotMass()*9.81);
    ihwbc_->setQPWeights(w_task_heirarchy_, local_w_contact_weight);
    ihwbc_->setRegularizationTerms(lambda_qddot_, lambda_Fr_);

    // Enable Torque Limits
    ihwbc_->enableTorqueLimits(true); 
    Eigen::VectorXd tau_min = robot_->GetTorqueLowerLimits().segment(Valkyrie::n_vdof, Valkyrie::n_adof);
    Eigen::VectorXd tau_max = robot_->GetTorqueUpperLimits().segment(Valkyrie::n_vdof, Valkyrie::n_adof);
    ihwbc_->setTorqueLimits(tau_min, tau_max);

    // QP dec variable results
    Eigen::VectorXd qddot_res;
    Eigen::VectorXd Fr_res;
    Eigen::VectorXd Fd_des = Eigen::VectorXd::Zero(dim_contact_);

    // Update QP and solve
    ihwbc_->updateSetting(A_, Ainv_, coriolis_, grav_);
    ihwbc_->solve(task_list_, contact_list_, Fd_des, tau_cmd_, qddot_cmd_);
    ihwbc_->getQddotResult(qddot_res);

    // Integrate Joint Velocities and Positions
    des_jacc_= qddot_res.segment(Valkyrie::n_vdof, Valkyrie::n_adof);
    ihwbc_joint_integrator_->integrate(des_jacc_, sp_->qdot.segment(Valkyrie::n_vdof, Valkyrie::n_adof),
                                                  sp_->q.segment(Valkyrie::n_vdof, Valkyrie::n_adof),
                                                  des_jvel_,
                                                  des_jpos_);

    // Eigen::VectorXd jpos_cur = sp_->q.segment(Valkyrie::n_vdof, Valkyrie::n_adof);
    // Eigen::VectorXd jvel_cur = sp_->qdot.segment(Valkyrie::n_vdof, Valkyrie::n_adof);
    // myUtils::pretty_print(des_jacc_, std::cout, "des_jacc_");
    // myUtils::pretty_print(des_jvel_, std::cout, "des_jvel_");
    // myUtils::pretty_print(jvel_cur, std::cout, "jvel_cur");

    // Test Wrench Frame values
    ihwbc_->getFrResult(Fr_res);
    // myUtils::pretty_print(Fr_res, std::cout, "Fr_res");
    Eigen::MatrixXd R_r_cop = robot_->getBodyNodeIsometry(ValkyrieBodyNode::rightCOP_Frame).linear();
    Eigen::MatrixXd R_l_cop = robot_->getBodyNodeIsometry(ValkyrieBodyNode::leftCOP_Frame).linear();
    // Compute local frame wrench values
    Eigen::MatrixXd Rot_foot(6, 6);
    Rot_foot.setZero();
    Rot_foot.block(0, 0, 3, 3) = R_r_cop.transpose();
    Rot_foot.block(3, 3, 3, 3) = R_r_cop.transpose();
    Eigen::VectorXd W_r_local = Rot_foot*Fr_res.head(6); 

    Rot_foot.block(0, 0, 3, 3) = R_l_cop.transpose();
    Rot_foot.block(3, 3, 3, 3) = R_l_cop.transpose();
    Eigen::VectorXd W_l_local = Rot_foot*Fr_res.tail(6);

    // myUtils::pretty_print(W_r_local, std::cout, "W_r_local_desired");
    // myUtils::pretty_print(W_l_local, std::cout, "W_l_local_desired");

    // TODO: Integration Step here

    gamma = tau_cmd_;

}

void DCMBalanceCtrl::_task_setup() {
    // =========================================================================
    // CoMxyzRxRyRzTask
    // =========================================================================
    //ori_traj
    //xyz_traj
    _GetBsplineTrajectory();
    // for com plotting
    for (int i = 0; i < 3; ++i) {
        (sp_->com_pos_des)[i] = des_com_pos_[i];
        (sp_->com_vel_des)[i] = des_com_vel_[i];
    }
    com_task_->updateTask(des_com_pos_.tail(3),des_com_vel_.tail(3),des_com_acc_.tail(3));

    // =========================================================================
    // Pelvis Ori Task
    // =========================================================================
    Eigen::VectorXd des_pelvis_quat = Eigen::VectorXd::Zero(4);
    des_pelvis_quat << ini_pelvis_quat_.w(),ini_pelvis_quat_.x(), ini_pelvis_quat_.y(),
                        ini_pelvis_quat_.z();
    Eigen::VectorXd des_pelvis_ang_vel = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd des_ang_acc = Eigen::VectorXd::Zero(3);
    pelvis_ori_task_ -> updateTask(des_pelvis_quat,des_pelvis_ang_vel,des_ang_acc);
    // =========================================================================
    // Joint Pos Task
    // =========================================================================
    Eigen::VectorXd jpos_des = sp_->jpos_ini;
    Eigen::VectorXd jvel_des(Valkyrie::n_adof);
    jvel_des.setZero();
    Eigen::VectorXd jacc_des(Valkyrie::n_adof);
    jacc_des.setZero();

    upper_body_task_->updateTask(jpos_des.tail(upper_body_joint_indices_.size()),
                                 jvel_des.tail(upper_body_joint_indices_.size()),
                                 jacc_des.tail(upper_body_joint_indices_.size()));

    // =========================================================================
    // Set Angular Momentum Tasks
    // =========================================================================
    Eigen::VectorXd zero3 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd des_ang_momentum = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd des_ang_momentum_rate = Eigen::VectorXd::Zero(3);
    ang_momentum_task_->updateTask(zero3, des_ang_momentum, des_ang_momentum_rate);

    // =========================================================================
    // Set Foot Motion Tasks
    // =========================================================================
    Eigen::VectorXd foot_pos_des(3); foot_pos_des.setZero();
    Eigen::VectorXd foot_vel_des(3); foot_vel_des.setZero();    
    Eigen::VectorXd foot_acc_des(3); foot_acc_des.setZero();    

    Eigen::VectorXd foot_ori_des(4); foot_ori_des.setZero();
    Eigen::VectorXd foot_ang_vel_des(3); foot_ang_vel_des.setZero();    
    Eigen::VectorXd foot_ang_acc_des(3); foot_ang_acc_des.setZero();

    // Set Right Foot Task
    foot_pos_des = robot_->getBodyNodeCoMIsometry(ValkyrieBodyNode::rightCOP_Frame).translation();
    Eigen::Quaternion<double> rfoot_ori_act(robot_->getBodyNodeCoMIsometry(ValkyrieBodyNode::rightCOP_Frame).linear());
    foot_ori_des[0] = rfoot_ori_act.w();
    foot_ori_des[1] = rfoot_ori_act.x();
    foot_ori_des[2] = rfoot_ori_act.y();
    foot_ori_des[3] = rfoot_ori_act.z();

    rfoot_center_pos_task->updateTask(foot_pos_des, foot_vel_des, foot_acc_des);
    rfoot_center_ori_task->updateTask(foot_ori_des, foot_ang_vel_des, foot_ang_acc_des);

    // Set Left Foot Task
    foot_pos_des = robot_->getBodyNodeCoMIsometry(ValkyrieBodyNode::leftCOP_Frame).translation();
    Eigen::Quaternion<double> lfoot_ori_act(robot_->getBodyNodeCoMIsometry(ValkyrieBodyNode::leftCOP_Frame).linear());
    foot_ori_des[0] = lfoot_ori_act.w();
    foot_ori_des[1] = lfoot_ori_act.x();
    foot_ori_des[2] = lfoot_ori_act.y();
    foot_ori_des[3] = lfoot_ori_act.z();

    lfoot_center_pos_task->updateTask(foot_pos_des, foot_vel_des, foot_acc_des);
    lfoot_center_ori_task->updateTask(foot_ori_des, foot_ang_vel_des, foot_ang_acc_des);

    // Set Tasks
    task_list_.clear();

    task_list_.push_back(com_task_);
    task_list_.push_back(pelvis_ori_task_);
    task_list_.push_back(upper_body_task_);

    task_list_.push_back(rfoot_center_pos_task);
    task_list_.push_back(rfoot_center_ori_task);
    task_list_.push_back(lfoot_center_pos_task);
    task_list_.push_back(lfoot_center_ori_task);

    task_list_.push_back(ang_momentum_task_);

    w_task_heirarchy_ = Eigen::VectorXd::Zero(task_list_.size());

    w_task_heirarchy_[0] = w_task_com_;
    w_task_heirarchy_[1] = w_task_pelvis_;
    w_task_heirarchy_[2] = w_task_upper_body_; 
    w_task_heirarchy_[3] = w_task_rfoot_;
    w_task_heirarchy_[4] = w_task_rfoot_;
    w_task_heirarchy_[5] = w_task_lfoot_;
    w_task_heirarchy_[6] = w_task_lfoot_;
    w_task_heirarchy_[7] = w_task_ang_mom_;

}

void DCMBalanceCtrl::_contact_setup() {
    rfoot_contact_->updateContactSpec();
    lfoot_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);

    for(int i = 0; i < contact_list_.size(); i++){
        ((SurfaceContactSpec*)contact_list_[i])->setMaxFz(1500);            
    }   

}

void DCMBalanceCtrl::firstVisit() {
    jpos_ini_ = sp_->q.segment(Valkyrie::n_vdof, Valkyrie::n_adof);
    ctrl_start_time_ = sp_->curr_time;

    ihwbc_joint_integrator_->initializeStates(Eigen::VectorXd::Zero(Valkyrie::n_adof), jpos_ini_);
    // ihwbc_joint_integrator_->printIntegrationParams();

    // ini_com_pos setting
    ini_com_pos_ = robot_ ->getCoMPosition();
    
    des_com_pos_ = ini_com_pos_;
    des_com_pos_.tail(3) = ini_com_pos_.tail(3) + com_pos_dev_;

    _SetBspline(ini_com_pos_.tail(3),des_com_pos_.tail(3));
    //myUtils::pretty_print(ini_com_pos_, std::cout, "ini");
    
    ini_pelvis_quat_ = Eigen::Quaternion<double>(robot_->getBodyNodeIsometry(ValkyrieBodyNode::pelvis).linear());
}

void DCMBalanceCtrl::lastVisit() {}

bool DCMBalanceCtrl::endOfPhase() {
     if (state_machine_time_ > end_time_) {
     std::cout << "end of DCM balance ctrl" << std::endl;
     return true;
    }
    return false;
}

void DCMBalanceCtrl::ctrlInitialization(const YAML::Node& node) {
    jpos_ini_ = sp_->q.segment(Valkyrie::n_vdof, Valkyrie::n_adof);
    try {
        myUtils::readParameter(node, "kp", Kp_);
        myUtils::readParameter(node, "kd", Kd_);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }

    // COM
    Eigen::VectorXd kp_com = 50*Eigen::VectorXd::Ones(3); 
    Eigen::VectorXd kd_com = 5.0*Eigen::VectorXd::Ones(3);

    // Ang Momentum
    Eigen::VectorXd kp_ang_mom = Eigen::VectorXd::Zero(3); 
    Eigen::VectorXd kd_ang_mom = 50.0*Eigen::VectorXd::Ones(3); //100.0*Eigen::VectorXd::Ones(3);

    // Pelvis
    Eigen::VectorXd kp_pelvis = 50*Eigen::VectorXd::Ones(3); 
    Eigen::VectorXd kd_pelvis = 5.0*Eigen::VectorXd::Ones(3);
    // Total Joint
    Eigen::VectorXd kp_joint = 50.0*Eigen::VectorXd::Ones(Valkyrie::n_adof); 
    Eigen::VectorXd kd_joint = 5.0*Eigen::VectorXd::Ones(Valkyrie::n_adof);

    // Upper Body Joint
    Eigen::VectorXd kp_upper_body_joint = 50.0*Eigen::VectorXd::Ones(upper_body_joint_indices_.size()); 
    Eigen::VectorXd kd_upper_body_joint = 5.0*Eigen::VectorXd::Ones(upper_body_joint_indices_.size());

    // Foot
    Eigen::VectorXd kp_foot = 50*Eigen::VectorXd::Ones(3); 
    Eigen::VectorXd kd_foot = 5.0*Eigen::VectorXd::Ones(3);

    // Set Task Gains
    com_task_->setGain(kp_com, kd_com);
    ang_momentum_task_->setGain(kp_ang_mom, kd_ang_mom);
    pelvis_ori_task_->setGain(kp_pelvis, kd_pelvis);
    upper_body_task_->setGain(kp_upper_body_joint, kd_upper_body_joint);

    rfoot_center_pos_task->setGain(kp_foot, kd_foot);
    rfoot_center_ori_task->setGain(kp_foot, kd_foot);
    lfoot_center_pos_task->setGain(kp_foot, kd_foot);
    lfoot_center_ori_task->setGain(kp_foot, kd_foot);

    // Set Hierarchy
    w_task_com_ = 5.0;
    w_task_pelvis_ = 5.0;

    w_task_ang_mom_ = 3.0;

    w_task_upper_body_ = 2.0;

    w_task_rfoot_ = 20.0;
    w_task_lfoot_ = 20.0;

    // Set Contact Weight
    w_contact_weight_ = 1e-3;

}

void DCMBalanceCtrl::_SetBspline(const Eigen::VectorXd st_pos,
                              const Eigen::VectorXd des_pos) {
    // Trajectory Setup
    double init[9];
    double fin[9];
    double** middle_pt = new double*[1];
    middle_pt[0] = new double[3];
    Eigen::Vector3d middle_pos;

    middle_pos = (st_pos + des_pos) / 2.;

    // Initial and final position & velocity & acceleration
    for (int i(0); i < 3; ++i) {
        // Initial
        init[i] = st_pos[i];
        init[i + 3] = 0.;
        init[i + 6] = 0.;
        // Final
        fin[i] = des_pos[i];
        fin[i + 3] = 0.;
        fin[i + 6] = 0.;
        // Middle
        middle_pt[0][i] = middle_pos[i];
    }
    // TEST
    //fin[5] = amplitude_[] * omega_;
    com_traj_.SetParam(init, fin, middle_pt, end_time_/2.0);

    delete[] * middle_pt;
    delete[] middle_pt;
}

void DCMBalanceCtrl::_GetBsplineTrajectory(){
    double pos[3];
    double vel[3];
    double acc[3];

    com_traj_.getCurvePoint(state_machine_time_, pos);
    com_traj_.getCurveDerPoint(state_machine_time_, 1, vel);
    com_traj_.getCurveDerPoint(state_machine_time_, 2, acc);

    for (int i(0); i < 3; ++i) {
        des_com_pos_[i] = pos[i];
        des_com_vel_[i] = vel[i];
        des_com_acc_[i] = acc[i];
    }
}
