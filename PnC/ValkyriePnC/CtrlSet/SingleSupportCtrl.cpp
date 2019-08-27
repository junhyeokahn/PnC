#include <array>

#include <PnC/ValkyriePnC/ContactSet/ContactSet.hpp>
#include <PnC/ValkyriePnC/CtrlSet/CtrlSet.hpp>
#include <PnC/ValkyriePnC/TaskSet/TaskSet.hpp>
#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>
#include <PnC/ValkyriePnC/ValkyrieInterface.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>
#include <PnC/WBC/WBLC/KinWBC.hpp>
#include <PnC/WBC/WBLC/WBLC.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/Math/MathUtilities.hpp>

SingleSupportCtrl::SingleSupportCtrl(RobotSystem* robot, Planner* planner,
                                     int moving_foot)
    : Controller(robot), planner_(planner), moving_foot_(moving_foot) {
    myUtils::pretty_constructor(2, "Single Support Ctrl");

    if (moving_foot_ == ValkyrieBodyNode::rightFoot) {
        moving_cop_ = ValkyrieBodyNode::rightCOP_Frame;
        stance_foot_ = ValkyrieBodyNode::leftFoot;
        stance_cop_ = ValkyrieBodyNode::leftCOP_Frame;
    } else {
        moving_cop_ = ValkyrieBodyNode::leftCOP_Frame;
        stance_foot_ = ValkyrieBodyNode::rightFoot;
        stance_cop_ = ValkyrieBodyNode::rightCOP_Frame;
    }
    ctrl_start_time_ = 0.;
    des_jpos_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    des_jvel_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    des_jacc_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    Kp_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    Kd_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);

    com_task_ = new BasicTask(robot, BasicTaskType::COM, 3);
    foot_pos_task_ =
        new BasicTask(robot, BasicTaskType::LINKXYZ, 3, moving_cop_);
    foot_ori_task_ =
        new BasicTask(robot, BasicTaskType::LINKORI, 3, moving_cop_);
    total_joint_task_ =
        new BasicTask(robot, BasicTaskType::JOINT, Valkyrie::n_adof);
    pelvis_ori_task_ = new BasicTask(robot, BasicTaskType::LINKORI, 3,
                                     ValkyrieBodyNode::pelvis);
    torso_ori_task_ = new BasicTask(robot, BasicTaskType::LINKORI, 3,
                                    ValkyrieBodyNode::torso);

    std::vector<bool> act_list;
    act_list.resize(Valkyrie::n_dof, true);
    for (int i(0); i < Valkyrie::n_vdof; ++i) act_list[i] = false;

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

    kin_wbc_contact_list_.clear();
    int rf_idx_offset(0);
    if (moving_foot_ == ValkyrieBodyNode::leftFoot) {
        rf_idx_offset = rfoot_contact_->getDim();
        for (int i(0); i < lfoot_contact_->getDim(); ++i) {
            wblc_data_->W_rf_[i + rf_idx_offset] = 5.0;
            wblc_data_->W_xddot_[i + rf_idx_offset] = 0.001;
        }
        wblc_data_->W_rf_[lfoot_contact_->getFzIndex() + rf_idx_offset] = 0.5;

        ((SurfaceContactSpec*)lfoot_contact_)->setMaxFz(0.0001);
        kin_wbc_contact_list_.push_back(rfoot_contact_);
    } else if (moving_foot_ == ValkyrieBodyNode::rightFoot) {
        for (int i(0); i < rfoot_contact_->getDim(); ++i) {
            wblc_data_->W_rf_[i + rf_idx_offset] = 5.0;
            wblc_data_->W_xddot_[i + rf_idx_offset] = 0.0001;
        }
        wblc_data_->W_rf_[rfoot_contact_->getFzIndex() + rf_idx_offset] = 0.5;

        ((SurfaceContactSpec*)rfoot_contact_)->setMaxFz(0.0001);
        kin_wbc_contact_list_.push_back(lfoot_contact_);
    } else
        printf("[Warnning] swing foot is not foot: %i\n", moving_foot_);
}

SingleSupportCtrl::~SingleSupportCtrl() {
    delete com_task_;
    delete foot_pos_task_;
    delete foot_ori_task_;
    delete total_joint_task_;
    delete pelvis_ori_task_;
    delete torso_ori_task_;

    delete kin_wbc_;
    delete wblc_;
    delete wblc_data_;

    delete rfoot_contact_;
    delete lfoot_contact_;
}

void SingleSupportCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;
    sp_->prev_state_machine_time = state_machine_time_;

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

void SingleSupportCtrl::PlannerInitialization_() {
    CentroidPlannerParameter* _param =
        ((CentroidPlanner*)planner_)->GetCentroidPlannerParameter();
    // =========================================================================
    // update initial state
    // =========================================================================
    Eigen::Vector3d r, l, k;
    r = robot_->getCoMPosition();
    Eigen::VectorXd lk = robot_->getCentroidMomentum();
    for (int i = 0; i < 3; ++i) {
        l[i] = lk[i + 3];
        k[i] = lk[i];
    }
    std::array<int, CentroidModel::numEEf> actv;
    std::array<Eigen::Vector3d, CentroidModel::numEEf> eef_frc;
    std::array<Eigen::Isometry3d, CentroidModel::numEEf> iso;
    std::array<Eigen::VectorXd, CentroidModel::numEEf> quat;
    for (int eef_id = 0; eef_id < CentroidModel::numEEf; ++eef_id) {
        actv[eef_id] = 0;
        eef_frc[eef_id].setZero();
    }
    if (moving_foot_ == ValkyrieBodyNode::rightFoot) {
        actv[static_cast<int>(CentroidModel::EEfID::rightFoot)] = 0;
        actv[static_cast<int>(CentroidModel::EEfID::leftFoot)] = 1;
        eef_frc[static_cast<int>(CentroidModel::EEfID::rightFoot)] << 0., 0.,
            0.;
        eef_frc[static_cast<int>(CentroidModel::EEfID::leftFoot)] << 0., 0.,
            1.0;
    } else {
        actv[static_cast<int>(CentroidModel::EEfID::rightFoot)] = 1;
        actv[static_cast<int>(CentroidModel::EEfID::leftFoot)] = 0;
        eef_frc[static_cast<int>(CentroidModel::EEfID::rightFoot)] << 0., 0.,
            1.;
        eef_frc[static_cast<int>(CentroidModel::EEfID::leftFoot)] << 0., 0., 0.;
    }

    iso[static_cast<int>(CentroidModel::EEfID::rightFoot)] =
        robot_->getBodyNodeIsometry(ValkyrieBodyNode::rightCOP_Frame);
    iso[static_cast<int>(CentroidModel::EEfID::leftFoot)] =
        robot_->getBodyNodeIsometry(ValkyrieBodyNode::leftCOP_Frame);
    iso[static_cast<int>(CentroidModel::EEfID::rightHand)] =
        robot_->getBodyNodeIsometry(ValkyrieBodyNode::rightPalm);
    iso[static_cast<int>(CentroidModel::EEfID::leftHand)] =
        robot_->getBodyNodeIsometry(ValkyrieBodyNode::leftPalm);

    for (int i = 0; i < CentroidModel::numEEf; ++i) {
        Eigen::VectorXd tmp_vec = Eigen::VectorXd::Zero(4);
        Eigen::Quaternion<double> tmp_quat =
            Eigen::Quaternion<double>(iso[i].linear());
        tmp_vec << tmp_quat.w(), tmp_quat.x(), tmp_quat.y(), tmp_quat.z();
        quat[i] = tmp_vec;
    }

    _param->UpdateInitialState(r, l, k, actv, eef_frc, iso);

    // =========================================================================
    // update reference dynamics state sequence
    // =========================================================================
    _param->UpdateRefDynamicsStateSequence();

    // =========================================================================
    // update contact plan interface
    // =========================================================================
    std::array<std::vector<Eigen::VectorXd>, CentroidModel::numEEf> c_seq;
    for (int i = 0; i < CentroidModel::numEEf; ++i) {
        c_seq[i].clear();
    }

    double rem_time(0.);
    rem_time = ssp_dur_ - state_machine_time_;
    double t_so_far(rem_time);
    int n_phase(1);
    while (true) {
        if (t_so_far >= _param->timeHorizon) break;
        if (n_phase % 2 == 1)
            t_so_far += dsp_dur_;
        else
            t_so_far += ssp_dur_;
        ++n_phase;
    }

    int n_con_seq(floor(n_phase / 2.0));

    Eigen::VectorXd rf_pos =
        iso[static_cast<int>(CentroidModel::EEfID::rightFoot)].translation();
    Eigen::VectorXd lf_pos =
        iso[static_cast<int>(CentroidModel::EEfID::leftFoot)].translation();
    Eigen::VectorXd r_ct_pos = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd l_ct_pos = Eigen::VectorXd::Zero(3);

    if (moving_foot_ == ValkyrieBodyNode::rightFoot) {
        // stance = left foot
        int n_rf_con_seq(ceil(n_phase / 2.0));
        int n_lf_con_seq(floor(n_phase / 2.0));
        double rf_st_time(rem_time);
        double lf_st_time(0.);
        double rf_end_time = rem_time + dsp_dur_ + ssp_dur_ + dsp_dur_;
        double lf_end_time = rem_time + dsp_dur_;
        // update rf contact sequence
        for (int i = 0; i < n_rf_con_seq; ++i) {
            r_ct_pos[0] = lf_pos[0] + footstep_length_ * (2 * i + 1);
            r_ct_pos[1] = lf_pos[1] - footstep_width_;
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)].push_back(
                Eigen::VectorXd::Zero(10));
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i][0] =
                rf_st_time;
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i][1] =
                rf_end_time;
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i]
                .segment<3>(2) = r_ct_pos;
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i]
                .segment<4>(5) = Eigen::VectorXd::Zero(4);
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i][5] = 1.;
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i][9] =
                static_cast<double>(ContactType::FlatContact);
            rf_st_time = rf_end_time + ssp_dur_;
            rf_end_time += ssp_dur_ + dsp_dur_ + ssp_dur_ + dsp_dur_;
            if (rf_st_time > _param->timeHorizon) break;
        }
        // update lf contact sequence
        l_ct_pos = lf_pos;
        for (int i = 0; i < n_lf_con_seq; ++i) {
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)].push_back(
                Eigen::VectorXd::Zero(10));
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i][0] =
                lf_st_time;
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i][1] =
                lf_end_time;
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i]
                .segment<3>(2) = l_ct_pos;
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i]
                .segment<4>(5) = Eigen::VectorXd::Zero(4);
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i][5] = 1.;
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i][9] =
                static_cast<double>(ContactType::FlatContact);
            lf_st_time = lf_end_time + ssp_dur_;
            lf_end_time += ssp_dur_ + dsp_dur_ + ssp_dur_ + dsp_dur_;
            if (lf_st_time > _param->timeHorizon) break;
            l_ct_pos[0] = lf_pos[0] + footstep_length_ * (2 * i + 2);
            l_ct_pos[1] = lf_pos[1];
        }
    } else {
        // stance = right foot
        int n_lf_con_seq(ceil(n_phase / 2.0));
        int n_rf_con_seq(floor(n_phase / 2.0));
        double lf_st_time(rem_time);
        double rf_st_time(0.);
        double lf_end_time = rem_time + dsp_dur_ + ssp_dur_ + dsp_dur_;
        double rf_end_time = rem_time + dsp_dur_;
        // update lf contact sequence
        Eigen::VectorXd l_ct_pos = Eigen::VectorXd::Zero(3);
        for (int i = 0; i < n_lf_con_seq; ++i) {
            l_ct_pos[0] = rf_pos[0] + footstep_length_ * (2 * i + 1);
            l_ct_pos[1] = rf_pos[1] + footstep_width_;
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)].push_back(
                Eigen::VectorXd::Zero(10));
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i][0] =
                lf_st_time;
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i][1] =
                lf_end_time;
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i]
                .segment<3>(2) = l_ct_pos;
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i]
                .segment<4>(5) = Eigen::VectorXd::Zero(4);
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i][5] = 1.;
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i][9] =
                static_cast<double>(ContactType::FlatContact);
            lf_st_time = lf_end_time + ssp_dur_;
            lf_end_time += ssp_dur_ + dsp_dur_ + ssp_dur_ + dsp_dur_;
            if (lf_st_time > _param->timeHorizon) break;
        }
        // update rf contact sequence
        r_ct_pos = rf_pos;
        for (int i = 0; i < n_rf_con_seq; ++i) {
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)].push_back(
                Eigen::VectorXd::Zero(10));
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i][0] =
                rf_st_time;
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i][1] =
                rf_end_time;
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i]
                .segment<3>(2) = r_ct_pos;
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i]
                .segment<4>(5) = Eigen::VectorXd::Zero(4);
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i][5] = 1.;
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i][9] =
                static_cast<double>(ContactType::FlatContact);
            rf_st_time = rf_end_time + ssp_dur_;
            rf_end_time += ssp_dur_ + dsp_dur_ + ssp_dur_ + dsp_dur_;
            if (rf_st_time > _param->timeHorizon) break;
            r_ct_pos[0] = rf_pos[0] + footstep_length_ * (2 * i + 2);
            r_ct_pos[1] = rf_pos[1];
        }
    }

    _param->UpdateContactPlanInterface(c_seq);

    // =========================================================================
    // update goal state
    // =========================================================================
    Eigen::Vector3d com_displacement;
    if (r_ct_pos[0] > l_ct_pos[0]) {
        for (int i = 0; i < 3; ++i) {
            com_displacement[i] = r_ct_pos[i] - r[i];
        }
    } else {
        for (int i = 0; i < 3; ++i) {
            com_displacement[i] = l_ct_pos[i] - r[i];
        }
    }
    com_displacement[2] = com_height_ - r[2];
    // com_displacement[2] = 0.;
    _param->UpdateTerminalState(com_displacement);

    // =========================================================================
    // save contact sequence for plotting
    // =========================================================================
    sp_->foot_target_list.clear();
    for (int eef_id = 0; eef_id < CentroidModel::numEEf; ++eef_id) {
        for (int c_id = 1; c_id < c_seq[eef_id].size(); ++c_id) {
            Eigen::Isometry3d tmp;
            tmp.translation() = c_seq[eef_id][c_id].segment<3>(2);
            tmp.linear() = Eigen::Quaternion<double>(
                               c_seq[eef_id][c_id][5], c_seq[eef_id][c_id][6],
                               c_seq[eef_id][c_id][7], c_seq[eef_id][c_id][8])
                               .toRotationMatrix();
            sp_->foot_target_list.push_back(tmp);
        }
    }
}

void SingleSupportCtrl::_compute_torque_wblc(Eigen::VectorXd& gamma) {
    // WBLC
    wblc_->updateSetting(A_, Ainv_, coriolis_, grav_);
    Eigen::VectorXd des_jacc_cmd =
        des_jacc_ +
        Kp_.cwiseProduct(des_jpos_ -
                         sp_->q.segment(Valkyrie::n_vdof, Valkyrie::n_adof)) +
        Kd_.cwiseProduct(des_jvel_ - sp_->qdot.tail(Valkyrie::n_adof));

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
    if (moving_foot_ == ValkyrieBodyNode::rightFoot) {
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

    if (moving_foot_ == ValkyrieBodyNode::rightFoot) {
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
    // Pelvis & Torso Ori Task
    // =========================================================================
    Eigen::Isometry3d rf_iso =
        robot_->getBodyNodeIsometry(ValkyrieBodyNode::rightCOP_Frame);
    Eigen::Isometry3d lf_iso =
        robot_->getBodyNodeIsometry(ValkyrieBodyNode::leftCOP_Frame);
    Eigen::Quaternion<double> rf_q = Eigen::Quaternion<double>(rf_iso.linear());
    Eigen::Quaternion<double> lf_q = Eigen::Quaternion<double>(lf_iso.linear());
    Eigen::Quaternion<double> des_q = rf_q.slerp(0.5, lf_q);
    Eigen::VectorXd des_quat_vec = Eigen::VectorXd::Zero(4);
    des_quat_vec << des_q.w(), des_q.x(), des_q.y(), des_q.z();

    Eigen::VectorXd des_so3 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd ori_acc_des = Eigen::VectorXd::Zero(3);

    pelvis_ori_task_->updateTask(des_quat_vec, des_so3, ori_acc_des);
    torso_ori_task_->updateTask(des_quat_vec, des_so3, ori_acc_des);

    // =========================================================================
    // Joint Pos Task
    // =========================================================================
    Eigen::VectorXd jpos_des = sp_->jpos_ini;
    Eigen::VectorXd jvel_des(Valkyrie::n_adof);
    jvel_des.setZero();
    Eigen::VectorXd jacc_des(Valkyrie::n_adof);
    jacc_des.setZero();
    total_joint_task_->updateTask(jpos_des, jvel_des, jacc_des);

    // =========================================================================
    // Task List Update
    // =========================================================================
    task_list_.push_back(com_task_);
    task_list_.push_back(pelvis_ori_task_);
    task_list_.push_back(torso_ori_task_);
    task_list_.push_back(foot_pos_task_);
    task_list_.push_back(foot_ori_task_);
    task_list_.push_back(total_joint_task_);

    // =========================================================================
    // Solve Inv Kinematics
    // =========================================================================
    kin_wbc_->FindConfiguration(sp_->q, task_list_, kin_wbc_contact_list_,
                                des_jpos_, des_jvel_, des_jacc_);
}

void SingleSupportCtrl::_contact_setup() {
    rfoot_contact_->updateContactSpec();
    lfoot_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);
}

void SingleSupportCtrl::firstVisit() {
    jpos_ini_ = sp_->q.segment(Valkyrie::n_vdof, Valkyrie::n_adof);
    ctrl_start_time_ = sp_->curr_time;
    SetBSpline_();

    ini_quat_pelvis_ = Eigen::Quaternion<double>(
        robot_->getBodyNodeIsometry(ValkyrieBodyNode::pelvis).linear());
    ini_quat_torso_ = Eigen::Quaternion<double>(
        robot_->getBodyNodeIsometry(ValkyrieBodyNode::torso).linear());

    if (moving_foot_ == ValkyrieBodyNode::rightFoot) {
        ini_quat_foot_ = Eigen::Quaternion<double>(
            robot_->getBodyNodeIsometry(ValkyrieBodyNode::rightCOP_Frame)
                .linear());
    } else {
        ini_quat_foot_ = Eigen::Quaternion<double>(
            robot_->getBodyNodeIsometry(ValkyrieBodyNode::leftCOP_Frame)
                .linear());
    }
    des_quat_ = Eigen::Quaternion<double>(sp_->moving_foot_target_iso.linear());

    if (sp_->num_residual_step == -1) {
        sp_->b_walking = false;
        std::cout << "stop at next ds" << std::endl;
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
    if (moving_foot_ == ValkyrieBodyNode::rightFoot) {
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
    jpos_ini_ = sp_->q.segment(Valkyrie::n_vdof, Valkyrie::n_adof);
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
