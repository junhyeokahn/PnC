#include <array>

#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/WBC/WBLC/KinWBC.hpp>
#include <PnC/WBC/WBLC/WBLC.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/Math/MathUtilities.hpp>

SingleSupportCtrl::SingleSupportCtrl(RobotSystem* robot, Planner* planner,
                                     int moving_foot)
    : Controller(robot), planner_(planner), moving_foot_(moving_foot) {
    myUtils::pretty_constructor(2, "Single Support Ctrl");

    if (moving_foot_ == DracoBodyNode::rAnkle) {
        moving_cop_ = DracoBodyNode::rFootCenter;
        stance_foot_ = DracoBodyNode::lAnkle;
        stance_cop_ = DracoBodyNode::lFootCenter;
    } else {
        moving_cop_ = DracoBodyNode::lFootCenter;
        stance_foot_ = DracoBodyNode::rAnkle;
        stance_cop_ = DracoBodyNode::rFootCenter;
    }
    ctrl_start_time_ = 0.;
    des_jpos_ = Eigen::VectorXd::Zero(Draco::n_adof);
    des_jvel_ = Eigen::VectorXd::Zero(Draco::n_adof);
    des_jacc_ = Eigen::VectorXd::Zero(Draco::n_adof);
    Kp_ = Eigen::VectorXd::Zero(Draco::n_adof);
    Kd_ = Eigen::VectorXd::Zero(Draco::n_adof);

    com_task_ = new BasicTask(robot, BasicTaskType::COM, 3);
    foot_pos_task_ =
        new BasicTask(robot, BasicTaskType::LINKXYZ, 3, moving_cop_);
    foot_ori_task_ =
        new BasicTask(robot, BasicTaskType::LINKORI, 3, moving_cop_);
    total_joint_task_ =
        new BasicTask(robot, BasicTaskType::JOINT, Draco::n_adof);
    torso_ori_task_ = new BasicTask(robot, BasicTaskType::LINKORI, 3,
                                    DracoBodyNode::Torso);

    std::vector<bool> act_list;
    act_list.resize(Draco::n_dof, true);
    for (int i(0); i < Draco::n_vdof; ++i) act_list[i] = false;

    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);
    wblc_data_ = new WBLC_ExtraData();
    //rfoot_contact_ = new SurfaceContactSpec(
        //robot_, DracoBodyNode::rFootCenter, 0.085, 0.02, 0.7);
    //lfoot_contact_ = new SurfaceContactSpec(
        //robot_, DracoBodyNode::lFootCenter, 0.085, 0.02, 0.7);
    //dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();

    rfoot_front_contact_ = new PointContactSpec(robot_, DracoBodyNode::rFootFront, 0.7);    
    rfoot_back_contact_ = new PointContactSpec(robot_, DracoBodyNode::rFootBack, 0.7);    
    lfoot_front_contact_ = new PointContactSpec(robot_, DracoBodyNode::lFootFront, 0.7);    
    lfoot_back_contact_ = new PointContactSpec(robot_, DracoBodyNode::lFootBack, 0.7);    

    dim_contact_ = rfoot_front_contact_->getDim() + lfoot_front_contact_->getDim() +
                    rfoot_back_contact_->getDim() + lfoot_back_contact_->getDim();


    wblc_data_->W_qddot_ = Eigen::VectorXd::Constant(Draco::n_dof, 100.0);
    wblc_data_->W_rf_ = Eigen::VectorXd::Constant(dim_contact_, 0.1);
    wblc_data_->W_xddot_ = Eigen::VectorXd::Constant(dim_contact_, 1000.0);
    //wblc_data_->W_rf_[rfoot_contact_->getFzIndex()] = 0.01;
    //wblc_data_->W_rf_[rfoot_contact_->getDim() + lfoot_contact_->getFzIndex()] =
        //0.01;

    wblc_data_->W_rf_[rfoot_front_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_front_contact_->getDim() + rfoot_back_contact_->getFzIndex()] =
        0.01;
    wblc_data_->W_rf_[rfoot_front_contact_->getDim() + rfoot_back_contact_->getDim() +
                lfoot_front_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_front_contact_->getDim() + rfoot_back_contact_->getDim() +
                lfoot_front_contact_->getDim() + lfoot_back_contact_->getFzIndex()] = 0.01;


    // torque limit default setting
    wblc_data_->tau_min_ = Eigen::VectorXd::Constant(Draco::n_adof, -2500.);
    wblc_data_->tau_max_ = Eigen::VectorXd::Constant(Draco::n_adof, 2500.);

    sp_ = DracoStateProvider::getStateProvider(robot);

    kin_wbc_contact_list_.clear();

    int rf_idx_offset(0);
    if (moving_foot_ == DracoBodyNode::lAnkle) {
        rf_idx_offset = rfoot_front_contact_->getDim() + rfoot_back_contact_->getDim();
        for (int i(0); i < lfoot_front_contact_->getDim() + lfoot_back_contact_->getDim(); ++i) {
            wblc_data_->W_rf_[i + rf_idx_offset] = 5.0;
            wblc_data_->W_xddot_[i + rf_idx_offset] = 0.001;
        }
        wblc_data_->W_rf_[lfoot_front_contact_->getFzIndex() + rf_idx_offset] = 0.5;
        wblc_data_->W_rf_[lfoot_front_contact_->getDim() + lfoot_back_contact_->getFzIndex() + rf_idx_offset] = 0.5;

        ((PointContactSpec*)lfoot_front_contact_)->setMaxFz(0.0001);
        ((PointContactSpec*)lfoot_back_contact_)->setMaxFz(0.0001);

        kin_wbc_contact_list_.push_back(rfoot_front_contact_);
        kin_wbc_contact_list_.push_back(rfoot_back_contact_);

    } else if (moving_foot_ == DracoBodyNode::rAnkle) {
        for (int i(0); i < rfoot_front_contact_->getDim() + rfoot_back_contact_->getDim(); ++i) {
            wblc_data_->W_rf_[i + rf_idx_offset] = 5.0;
            wblc_data_->W_xddot_[i + rf_idx_offset] = 0.0001;
        }
        wblc_data_->W_rf_[rfoot_front_contact_->getFzIndex() + rf_idx_offset] = 0.5;
        wblc_data_->W_rf_[rfoot_front_contact_->getDim() + rfoot_back_contact_->getFzIndex() + rf_idx_offset] = 0.5;

        ((PointContactSpec*)rfoot_front_contact_)->setMaxFz(0.0001);
        ((PointContactSpec*)rfoot_back_contact_)->setMaxFz(0.0001);
        kin_wbc_contact_list_.push_back(lfoot_front_contact_);
        kin_wbc_contact_list_.push_back(lfoot_back_contact_);

    } else
        printf("[Warnning] swing foot is not foot: %i\n", moving_foot_);
}

SingleSupportCtrl::~SingleSupportCtrl() {
    delete com_task_;
    delete foot_pos_task_;
    delete foot_ori_task_;
    delete total_joint_task_;
    delete torso_ori_task_;

    delete kin_wbc_;
    delete wblc_;
    delete wblc_data_;

    //delete rfoot_contact_;
    //delete lfoot_contact_;

    delete rfoot_front_contact_;
    delete rfoot_back_contact_;
    delete lfoot_front_contact_;
    delete lfoot_back_contact_;
}

void SingleSupportCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;
    sp_->prev_state_machine_time = state_machine_time_;

    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(Draco::n_adof);
    _contact_setup();
    _task_setup();
    _compute_torque_wblc(gamma);

    for (int i(0); i < Draco::n_adof; ++i) {
        ((DracoCommand*)_cmd)->jtrq[i] = gamma[i];
        ((DracoCommand*)_cmd)->q[i] = des_jpos_[i];
        ((DracoCommand*)_cmd)->qdot[i] = des_jvel_[i];
    }

    _PostProcessing_Command();
}

void SingleSupportCtrl::_compute_torque_wblc(Eigen::VectorXd& gamma) {
    // WBLC
    wblc_->updateSetting(A_, Ainv_, coriolis_, grav_);
    Eigen::VectorXd des_jacc_cmd =
        des_jacc_ +
        Kp_.cwiseProduct(des_jpos_ -
                         sp_->q.segment(Draco::n_vdof, Draco::n_adof)) +
        Kd_.cwiseProduct(des_jvel_ - sp_->qdot.tail(Draco::n_adof));

    sp_->des_jacc_cmd = des_jacc_cmd;
    wblc_->makeWBLC_Torque(des_jacc_cmd, contact_list_, gamma, wblc_data_);
    sp_->r_rf_des = wblc_data_->Fr_.head(6);
    sp_->l_rf_des = wblc_data_->Fr_.tail(6);
}

void SingleSupportCtrl::_task_setup() {
    // =========================================================================
    // COM Task
    // =========================================================================

    Eigen::VectorXd cen_pos_des = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd cen_vel_des = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd cen_acc_des = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd dummy = Eigen::VectorXd::Zero(6);
    planner_->EvalTrajectory(sp_->curr_time - sp_->planning_moment, cen_pos_des,
                             cen_vel_des, dummy);

    Eigen::VectorXd com_pos_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd com_vel_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd com_acc_des = Eigen::VectorXd::Zero(3);

    for (int i = 0; i < 3; ++i) {
        com_pos_des[i] = cen_pos_des[i + 3];
        com_vel_des[i] = cen_vel_des[i + 3] / robot_->getRobotMass();
    }

    for (int i = 0; i < 3; ++i) {
        sp_->com_pos_des[i] = com_pos_des[i];
        sp_->com_vel_des[i] = com_vel_des[i];
        sp_->mom_des[i] = cen_vel_des[i];
        sp_->mom_des[i + 3] = cen_vel_des[i + 3];
    }

    com_task_->updateTask(com_pos_des, com_vel_des, com_acc_des);

    // =========================================================================
    // Foot Pos Task
    // =========================================================================
    Eigen::VectorXd foot_pos_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd foot_vel_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd foot_acc_des = Eigen::VectorXd::Zero(3);

    double pos[3];
    double vel[3];
    double acc[3];
    foot_traj_.getCurvePoint(state_machine_time_, pos);
    foot_traj_.getCurveDerPoint(state_machine_time_, 1, vel);
    foot_traj_.getCurveDerPoint(state_machine_time_, 2, acc);

    for (int i(0); i < 3; ++i) {
        foot_pos_des[i] = pos[i];
        foot_vel_des[i] = vel[i];
        foot_acc_des[i] = acc[i];
    }
    if (moving_foot_ == DracoBodyNode::rAnkle) {
        for (int i = 0; i < 3; ++i) {
            sp_->rf_pos_des[i] = foot_pos_des[i];
            sp_->rf_vel_des[i] = foot_vel_des[i];
        }
    } else {
        for (int i = 0; i < 3; ++i) {
            sp_->lf_pos_des[i] = foot_pos_des[i];
            sp_->lf_vel_des[i] = foot_vel_des[i];
        }
    }

    foot_pos_task_->updateTask(foot_pos_des, foot_vel_des, foot_acc_des);

    // =========================================================================
    // Foot Ori Task
    // =========================================================================
    double t = myUtils::smooth_changing(0, 1, ssp_dur_, state_machine_time_);
    double tdot =
        myUtils::smooth_changing_vel(0, 1, ssp_dur_, state_machine_time_);

    Eigen::Quaternion<double> quat_ori_error =
        des_quat_ * (ini_quat_foot_.inverse());
    Eigen::VectorXd ang_vel = Eigen::VectorXd::Zero(3);
    ang_vel = dart::math::quatToExp(quat_ori_error);

    Eigen::VectorXd ori_increment = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd curr_ang_vel_des = Eigen::VectorXd::Zero(3);
    Eigen::Quaternion<double> quat_increment;
    Eigen::Quaternion<double> curr_quat_des;
    ori_increment = ang_vel * t;
    quat_increment = dart::math::expToQuat(ori_increment);
    curr_quat_des = quat_increment * ini_quat_foot_;

    curr_ang_vel_des = ang_vel * tdot;

    Eigen::VectorXd quat_foot_des = Eigen::VectorXd::Zero(4);
    quat_foot_des << curr_quat_des.w(), curr_quat_des.x(), curr_quat_des.y(),
        curr_quat_des.z();
    Eigen::VectorXd ang_acc_des = Eigen::VectorXd::Zero(3);

    if (moving_foot_ == DracoBodyNode::rAnkle) {
        sp_->rf_ori_quat_des = curr_quat_des;
        for (int i = 0; i < 3; ++i) {
            sp_->rf_ang_vel_des[i] = curr_ang_vel_des[i];
        }
    } else {
        sp_->lf_ori_quat_des = curr_quat_des;
        for (int i = 0; i < 3; ++i) {
            sp_->lf_ang_vel_des[i] = curr_ang_vel_des[i];
        }
    }

    foot_ori_task_->updateTask(quat_foot_des, curr_ang_vel_des, ang_acc_des);

    // =========================================================================
    // Torso Ori Task
    // =========================================================================
    Eigen::Isometry3d rf_iso =
        robot_->getBodyNodeIsometry(DracoBodyNode::rFootCenter);
    Eigen::Isometry3d lf_iso =
        robot_->getBodyNodeIsometry(DracoBodyNode::lFootCenter);
    Eigen::Quaternion<double> rf_q = Eigen::Quaternion<double>(rf_iso.linear());
    Eigen::Quaternion<double> lf_q = Eigen::Quaternion<double>(lf_iso.linear());
    Eigen::Quaternion<double> des_q = rf_q.slerp(0.5, lf_q);
    Eigen::VectorXd des_quat_vec = Eigen::VectorXd::Zero(4);
    des_quat_vec << des_q.w(), des_q.x(), des_q.y(), des_q.z();

    Eigen::VectorXd des_so3 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd ori_acc_des = Eigen::VectorXd::Zero(3);

    torso_ori_task_->updateTask(des_quat_vec, des_so3, ori_acc_des);

    // =========================================================================
    // Joint Pos Task
    // =========================================================================
    Eigen::VectorXd jpos_des = sp_->jpos_ini;
    Eigen::VectorXd jvel_des(Draco::n_adof);
    jvel_des.setZero();
    Eigen::VectorXd jacc_des(Draco::n_adof);
    jacc_des.setZero();
    total_joint_task_->updateTask(jpos_des, jvel_des, jacc_des);

    // =========================================================================
    // Task List Update
    // =========================================================================
    task_list_.push_back(com_task_);
    task_list_.push_back(torso_ori_task_);
    task_list_.push_back(foot_pos_task_);
    task_list_.push_back(foot_ori_task_);
    task_list_.push_back(total_joint_task_);

    //task_list_.push_back(com_task_);
    //task_list_.push_back(foot_pos_task_);
    //task_list_.push_back(torso_ori_task_);
    //task_list_.push_back(foot_ori_task_);
    //task_list_.push_back(total_joint_task_);
    // =========================================================================
    // Solve Inv Kinematics
    // =========================================================================
    kin_wbc_->FindConfiguration(sp_->q, task_list_, kin_wbc_contact_list_,
                                des_jpos_, des_jvel_, des_jacc_);
}

void SingleSupportCtrl::_contact_setup() {
    rfoot_front_contact_->updateContactSpec();
    rfoot_back_contact_->updateContactSpec();
    lfoot_front_contact_->updateContactSpec();
    lfoot_back_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_front_contact_);
    contact_list_.push_back(rfoot_back_contact_);
    contact_list_.push_back(lfoot_front_contact_);
    contact_list_.push_back(lfoot_back_contact_);
}

void SingleSupportCtrl::firstVisit() {
    std::cout << "singlesupport" << std::endl;
    jpos_ini_ = sp_->q.segment(Draco::n_vdof, Draco::n_adof);
    ctrl_start_time_ = sp_->curr_time;
    SetBSpline_();

    ini_quat_torso_ = Eigen::Quaternion<double>(
        robot_->getBodyNodeIsometry(DracoBodyNode::Torso).linear());

    if (moving_foot_ == DracoBodyNode::rAnkle) {
        ini_quat_foot_ = Eigen::Quaternion<double>(
            robot_->getBodyNodeIsometry(DracoBodyNode::rFootCenter)
                .linear());
    } else {
        ini_quat_foot_ = Eigen::Quaternion<double>(
            robot_->getBodyNodeIsometry(DracoBodyNode::lFootCenter)
                .linear());
    }
    des_quat_ = Eigen::Quaternion<double>(sp_->moving_foot_target_iso.linear());

    if (sp_->num_residual_step == -1) {
        sp_->b_walking = false;
    }
}

void SingleSupportCtrl::SetBSpline_() {
    double ini[9];
    double fin[9];
    double** middle_pt = new double*[1];
    middle_pt[0] = new double[3];

    Eigen::VectorXd ini_pos =
        robot_->getBodyNodeIsometry(moving_cop_).translation();
    Eigen::VectorXd fin_pos = sp_->moving_foot_target_iso.translation();

    for (int i = 0; i < 3; ++i) {
        ini[i] = ini_pos[i];
        ini[i + 3] = 0.;
        ini[i + 6] = 0.;

        fin[i] = fin_pos[i];
        fin[i + 3] = 0.;
        fin[i + 6] = 0.;

        middle_pt[0][i] = 0.;
    }
    for (int i = 0; i < 2; ++i) {
        middle_pt[0][i] = (ini_pos[i] + fin_pos[i]) / 2.0;
    }
    middle_pt[0][2] = swing_height_;
    fin[5] = -0.5;
    fin[8] = 5.;
    foot_traj_.SetParam(ini, fin, middle_pt, ssp_dur_);

    delete[] * middle_pt;
    delete[] middle_pt;
}

void SingleSupportCtrl::lastVisit() {}

bool SingleSupportCtrl::endOfPhase() {
    if (state_machine_time_ > ssp_dur_) {
        printf("(state_machine time, end time) : (%f, %f) \n",
               state_machine_time_, ssp_dur_);
        return true;
    }
    if (moving_foot_ == DracoBodyNode::rAnkle) {
        if (state_machine_time_ > ssp_dur_ * 0.5 && sp_->b_rfoot_contact) {
            printf("(state_machine time, end time) : (%f, %f) \n",
                   state_machine_time_, ssp_dur_);
            return true;
        }
    } else {
        if (state_machine_time_ > ssp_dur_ * 0.5 && sp_->b_lfoot_contact) {
            printf("(state_machine time, end time) : (%f, %f) \n",
                   state_machine_time_, ssp_dur_);
            return true;
        }
    }
    return false;
}

void SingleSupportCtrl::ctrlInitialization(const YAML::Node& node) {
    jpos_ini_ = sp_->q.segment(Draco::n_vdof, Draco::n_adof);
    try {
        myUtils::readParameter(node, "kp", Kp_);
        myUtils::readParameter(node, "kd", Kd_);

        Eigen::VectorXd tmp_vec1, tmp_vec2;
        myUtils::readParameter(node, "com_kp", tmp_vec1);
        myUtils::readParameter(node, "com_kd", tmp_vec2);
        com_task_->setGain(tmp_vec1, tmp_vec2);

        myUtils::readParameter(node, "foot_pos_kp", tmp_vec1);
        myUtils::readParameter(node, "foot_pos_kd", tmp_vec2);
        foot_pos_task_->setGain(tmp_vec1, tmp_vec2);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}
