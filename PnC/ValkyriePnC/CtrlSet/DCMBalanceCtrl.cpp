#include <PnC/ValkyriePnC/ContactSet/ContactSet.hpp>
#include <PnC/ValkyriePnC/CtrlSet/CtrlSet.hpp>
#include <PnC/ValkyriePnC/TaskSet/TaskSet.hpp>
#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>
#include <PnC/ValkyriePnC/ValkyrieInterface.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>
#include <PnC/WBC/IHWBC/IHWBC.hpp>
#include <PnC/WBC/WBLC/KinWBC.hpp>
#include <PnC/WBC/WBLC/WBLC.hpp>
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

    ini_com_pos_ = Eigen::VectorXd::Zero(7);
    des_com_pos_ = Eigen::VectorXd::Zero(7);
    des_com_vel_ = Eigen::VectorXd::Zero(6);
    des_com_acc_ = Eigen::VectorXd::Zero(6);

    com_pos_dev_ = Eigen::VectorXd::Zero(3);

    ini_torso_quat = Eigen::Quaternion<double> (1,0,0,0);
    // TASK
    com_task_ =
        new CoMxyzRxRyRzTask(robot);
    pelvis_ori_task_ =
        new BasicTask(robot,BasicTaskType::LINKORI, 3, ValkyrieBodyNode::pelvis);
    total_joint_task_ =
        new BasicTask(robot, BasicTaskType::JOINT, Valkyrie::n_adof);

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

    std::vector<bool> act_list;
    act_list.resize(Valkyrie::n_dof, true);
    for (int i(0); i < Valkyrie::n_vdof; ++i) act_list[i] = false;

    ihwbc_ = new IHWBC(act_list);

    lambda_qddot_ = 1e-8;
    lambda_Fr_ = 1e-16;

    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);
    wblc_data_ = new WBLC_ExtraData();
    rfoot_contact_ = new SurfaceContactSpec(
        robot_, ValkyrieBodyNode::rightCOP_Frame, 0.135, 0.08, 0.7);
    lfoot_contact_ = new SurfaceContactSpec(
        robot_, ValkyrieBodyNode::leftCOP_Frame, 0.135, 0.08, 0.7);
    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();

    wblc_data_->W_qddot_ = Eigen::VectorXd::Constant(Valkyrie::n_dof, 100.0);
    wblc_data_->W_rf_ = Eigen::VectorXd::Constant(dim_contact_, 0.1);
    wblc_data_->W_xddot_ = Eigen::VectorXd::Constant(dim_contact_, 1000.0);
    wblc_data_->W_rf_[rfoot_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_contact_->getDim() + lfoot_contact_->getFzIndex()] =
        0.01;

    // torque limit default setting
    wblc_data_->tau_min_ = Eigen::VectorXd::Constant(Valkyrie::n_adof, -2500.);
    wblc_data_->tau_max_ = Eigen::VectorXd::Constant(Valkyrie::n_adof, 2500.);

    sp_ = ValkyrieStateProvider::getStateProvider(robot);

}

DCMBalanceCtrl::~DCMBalanceCtrl() {
    delete total_joint_task_;
    delete com_task_;
    delete pelvis_ori_task_;
    delete wblc_;
    delete wblc_data_;
    delete rfoot_contact_;
    delete lfoot_contact_;
}

void DCMBalanceCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;

    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    _contact_setup();
    _task_setup();
    _compute_torque_wblc(gamma);

    for (int i(0); i < Valkyrie::n_adof; ++i) {
        ((ValkyrieCommand*)_cmd)->jtrq[i] = gamma[i];
        ((ValkyrieCommand*)_cmd)->q[i] = des_jpos_[i];
        ((ValkyrieCommand*)_cmd)->qdot[i] = des_jvel_[i];
    }

    _PostProcessing_Command();
}

void DCMBalanceCtrl::_compute_torque_wblc(Eigen::VectorXd& gamma) {
    // WBLC
    wblc_->updateSetting(A_, Ainv_, coriolis_, grav_);
    Eigen::VectorXd des_jacc_cmd =
        des_jacc_ +
        Kp_.cwiseProduct(des_jpos_ -
                         sp_->q.segment(Valkyrie::n_vdof, Valkyrie::n_adof)) +
        Kd_.cwiseProduct(des_jvel_ - sp_->qdot.tail(Valkyrie::n_adof));

    // myUtils::pretty_print(des_jacc_cmd, std::cout, "balance");
    // exit(0);
    wblc_->makeWBLC_Torque(des_jacc_cmd, contact_list_, gamma, wblc_data_);

    // Set QP weights
    double local_w_contact_weight = w_contact_weight_/(robot_->getRobotMass()*9.81);
    ihwbc_->setQPWeights(w_task_heirarchy_, local_w_contact_weight);
    ihwbc_->setRegularizationTerms(lambda_qddot_, lambda_Fr_);

    // QP dec variable results
    Eigen::VectorXd qddot_res;
    Eigen::VectorXd Fr_res;
    Eigen::VectorXd Fd_des = Eigen::VectorXd::Zero(dim_contact_);

    // Update QP and solve
    ihwbc_->updateSetting(A_, Ainv_, coriolis_, grav_);
    ihwbc_->solve(task_list_, contact_list_, Fd_des, tau_cmd_, qddot_cmd_);

    ihwbc_->getQddotResult(qddot_res);
    ihwbc_->getFrResult(Fr_res);

    // myUtils::pretty_print(Fr_res, std::cout, "Fr_des");
    // myUtils::pretty_print(tau_cmd_, std::cout, "tau_cmd_");


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
        (sp_->com_pos_des)[i] = des_com_pos_[i+4];
        (sp_->com_vel_des)[i] = des_com_vel_[i+3];
    }
    com_task_->updateTask(des_com_pos_,des_com_vel_,des_com_acc_);

    // =========================================================================
    // Pelvis Ori Task
    // =========================================================================
    Eigen::VectorXd des_torso_quat = Eigen::VectorXd::Zero(4);
    des_torso_quat << ini_torso_quat.w(),ini_torso_quat.x(), ini_torso_quat.y(),
                        ini_torso_quat.z();
    Eigen::VectorXd des_torso_so3 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd des_ang_acc = Eigen::VectorXd::Zero(3);
    pelvis_ori_task_ -> updateTask(des_torso_quat,des_torso_so3,des_ang_acc);
    // =========================================================================
    // Joint Pos Task
    // =========================================================================
    Eigen::VectorXd jpos_des = sp_->jpos_ini;
    Eigen::VectorXd jvel_des(Valkyrie::n_adof);
    jvel_des.setZero();
    Eigen::VectorXd jacc_des(Valkyrie::n_adof);
    jacc_des.setZero();
    total_joint_task_->updateTask(jpos_des, jvel_des, jacc_des);

    upper_body_task_->updateTask(jpos_des.tail(upper_body_joint_indices_.size()),
                                 jvel_des.tail(upper_body_joint_indices_.size()),
                                 jacc_des.tail(upper_body_joint_indices_.size()));

    // =========================================================================
    // Task List Update
    // =========================================================================
    task_list_.push_back(com_task_);
    task_list_.push_back(pelvis_ori_task_);
    task_list_.push_back(total_joint_task_);

    // =========================================================================
    // Solve Inv Kinematics
    // =========================================================================
    kin_wbc_->FindConfiguration(sp_->q, task_list_, contact_list_, des_jpos_,
                                des_jvel_, des_jacc_);

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

    task_list_.clear();

    task_list_.push_back(com_task_);
    task_list_.push_back(pelvis_ori_task_);
    task_list_.push_back(upper_body_task_);

    task_list_.push_back(rfoot_center_pos_task);
    task_list_.push_back(rfoot_center_ori_task);
    task_list_.push_back(lfoot_center_pos_task);
    task_list_.push_back(lfoot_center_ori_task);

    w_task_heirarchy_ = Eigen::VectorXd::Zero(task_list_.size());

    w_task_heirarchy_[0] = w_task_com_;
    w_task_heirarchy_[1] = w_task_pelvis_;
    w_task_heirarchy_[2] = w_task_upper_body_; 
    w_task_heirarchy_[3] = w_task_rfoot_;
    w_task_heirarchy_[4] = w_task_rfoot_;
    w_task_heirarchy_[5] = w_task_lfoot_;
    w_task_heirarchy_[6] = w_task_lfoot_;

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

    // ini_com_pos setting
    Eigen::Quaternion<double> com_pos_ori_ini(robot_->getBodyNodeIsometry(ValkyrieBodyNode::pelvis).linear());
    ini_com_pos_ <<  com_pos_ori_ini.w(), com_pos_ori_ini.x(),
                    com_pos_ori_ini.y(), com_pos_ori_ini.z();

    Eigen::Vector3d com_pos_xyz_ini = robot_ ->getCoMPosition();
    for (int i(0); i < 3; ++i) {
        ini_com_pos_[i+4] = com_pos_xyz_ini[i]; 
    }
    
    des_com_pos_ = ini_com_pos_;
    des_com_pos_.tail(3) = ini_com_pos_.tail(3) + com_pos_dev_;

    _SetBspline(ini_com_pos_.tail(3),des_com_pos_.tail(3));
    //myUtils::pretty_print(ini_com_pos_, std::cout, "ini");
    
    ini_torso_quat = Eigen::Quaternion<double>(robot_->getBodyNodeIsometry(ValkyrieBodyNode::torso).linear());
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
    Eigen::VectorXd kp_com = 1000*Eigen::VectorXd::Ones(6); 
    Eigen::VectorXd kd_com = 100.0*Eigen::VectorXd::Ones(6);
    kp_com.head(3) = Eigen::VectorXd::Zero(3);
    kd_com.head(3) = Eigen::VectorXd::Zero(3);

    // Pelvis
    Eigen::VectorXd kp_pelvis = 100*Eigen::VectorXd::Ones(3); 
    Eigen::VectorXd kd_pelvis = 10.0*Eigen::VectorXd::Ones(3);
    // Total Joint
    Eigen::VectorXd kp_joint = 100.0*Eigen::VectorXd::Ones(Valkyrie::n_adof); 
    Eigen::VectorXd kd_joint = 10.0*Eigen::VectorXd::Ones(Valkyrie::n_adof);

    // Upper Body Joint
    Eigen::VectorXd kp_upper_body_joint = 100.0*Eigen::VectorXd::Ones(upper_body_joint_indices_.size()); 
    Eigen::VectorXd kd_upper_body_joint = 10.0*Eigen::VectorXd::Ones(upper_body_joint_indices_.size());

    // Foot
    Eigen::VectorXd kp_foot = 100*Eigen::VectorXd::Ones(3); 
    Eigen::VectorXd kd_foot = 10.0*Eigen::VectorXd::Ones(3);

    // Set Task Gains
    com_task_->setGain(kp_com, kd_com);
    pelvis_ori_task_->setGain(kp_pelvis, kd_pelvis);
    total_joint_task_->setGain(kp_joint, kd_joint);
    upper_body_task_->setGain(kp_upper_body_joint, kd_upper_body_joint);

    rfoot_center_pos_task->setGain(kp_foot, kd_foot);
    rfoot_center_ori_task->setGain(kp_foot, kd_foot);
    lfoot_center_pos_task->setGain(kp_foot, kd_foot);
    lfoot_center_ori_task->setGain(kp_foot, kd_foot);

    // Set Hierarchy
    w_task_com_ = 5.0;
    w_task_pelvis_ = 5.0;
    w_task_joint_ = 1e-1;

    w_task_upper_body_ = 2.0;

    w_task_rfoot_ = 100.0;
    w_task_lfoot_ = 100.0;

    // Set Contact Weight
    w_contact_weight_ = 0.0;

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
        des_com_pos_[i+4] = pos[i];
        des_com_vel_[i+3] = vel[i];
        des_com_acc_[i+3] = acc[i];
    }
}
