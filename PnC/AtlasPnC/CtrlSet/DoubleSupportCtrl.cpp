#include <array>

#include <PnC/AtlasPnC/AtlasDefinition.hpp>
#include <PnC/AtlasPnC/AtlasInterface.hpp>
#include <PnC/AtlasPnC/AtlasStateProvider.hpp>
#include <PnC/AtlasPnC/ContactSet/ContactSet.hpp>
#include <PnC/AtlasPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/AtlasPnC/TaskSet/TaskSet.hpp>
#include <PnC/PlannerSet/CentroidPlanner/CentroidPlanner.hpp>
#include <PnC/WBC/WBLC/KinWBC.hpp>
#include <PnC/WBC/WBLC/WBLC.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/Math/MathUtilities.hpp>

DoubleSupportCtrl::DoubleSupportCtrl(RobotSystem* robot, Planner* planner)
    : Controller(robot), planner_(planner) {
    myUtils::pretty_constructor(2, "Double Support Ctrl");
    dsp_dur_ = 1000.;
    ctrl_start_time_ = 0.;
    des_jpos_ = Eigen::VectorXd::Zero(Atlas::n_adof);
    des_jvel_ = Eigen::VectorXd::Zero(Atlas::n_adof);
    des_jacc_ = Eigen::VectorXd::Zero(Atlas::n_adof);
    Kp_ = Eigen::VectorXd::Zero(Atlas::n_adof);
    Kd_ = Eigen::VectorXd::Zero(Atlas::n_adof);

    total_joint_task_ =
        new BasicTask(robot, BasicTaskType::JOINT, Atlas::n_adof);

    std::vector<bool> act_list;
    act_list.resize(Atlas::n_dof, true);
    for (int i(0); i < Atlas::n_vdof; ++i) act_list[i] = false;

    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);
    wblc_data_ = new WBLC_ExtraData();
    rfoot_contact_ = new SurfaceContactSpec(robot_, AtlasBodyNode::r_sole,
                                            0.125, 0.075, 0.9);
    lfoot_contact_ = new SurfaceContactSpec(robot_, AtlasBodyNode::l_sole,
                                            0.125, 0.075, 0.9);
    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();

    wblc_data_->W_qddot_ = Eigen::VectorXd::Constant(Atlas::n_dof, 100.0);
    wblc_data_->W_rf_ = Eigen::VectorXd::Constant(dim_contact_, 0.1);
    wblc_data_->W_xddot_ = Eigen::VectorXd::Constant(dim_contact_, 1000.0);
    wblc_data_->W_rf_[rfoot_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_contact_->getDim() + lfoot_contact_->getFzIndex()] =
        0.01;

    // torque limit default setting
    wblc_data_->tau_min_ = Eigen::VectorXd::Constant(Atlas::n_adof, -2500.);
    wblc_data_->tau_max_ = Eigen::VectorXd::Constant(Atlas::n_adof, 2500.);

    sp_ = AtlasStateProvider::getStateProvider(robot);
}

DoubleSupportCtrl::~DoubleSupportCtrl() {
    delete total_joint_task_;
    delete wblc_;
    delete wblc_data_;
    delete rfoot_contact_;
    delete lfoot_contact_;
}

void DoubleSupportCtrl::oneStep(void* _cmd) {
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;

    UpdateMPC_();
    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(Atlas::n_adof);
    _contact_setup();
    _task_setup();
    _compute_torque_wblc(gamma);

    for (int i(0); i < Atlas::n_adof; ++i) {
        ((AtlasCommand*)_cmd)->jtrq[i] = gamma[i];
        ((AtlasCommand*)_cmd)->q[i] = des_jpos_[i];
        ((AtlasCommand*)_cmd)->qdot[i] = des_jvel_[i];
    }

    _PostProcessing_Command();
}

void DoubleSupportCtrl::UpdateMPC_() {
    PlannerInitialization_();
    planner_->DoPlan();
    exit(0);
}

void DoubleSupportCtrl::PlannerInitialization_() {
    CentroidPlannerParameter* _param =
        ((CentroidPlanner*)planner_)->GetCentroidPlannerParameter();
    // =========================================================================
    // update initial state
    // =========================================================================
    Eigen::Vector3d r, l, k;
    r = robot_->getCoMPosition() + sp_->global_pos_local;
    Eigen::VectorXd lk = robot_->getCentroidMomentum();
    for (int i = 0; i < 3; ++i) {
        l[i] = lk[i];
        k[i] = lk[i + 3];
    }
    std::array<int, CentroidModel::numEEf> actv;
    std::array<Eigen::Vector3d, CentroidModel::numEEf> eef_frc;
    std::array<Eigen::Isometry3d, CentroidModel::numEEf> iso;
    std::array<Eigen::VectorXd, CentroidModel::numEEf> quat;
    for (int eef_id = 0; eef_id < CentroidModel::numEEf; ++eef_id) {
        actv[eef_id] = 0;
        eef_frc[eef_id].setZero();
    }
    actv[static_cast<int>(CentroidModel::EEfID::rightFoot)] = 1;
    actv[static_cast<int>(CentroidModel::EEfID::leftFoot)] = 1;
    // TODO : reaction force distribution
    eef_frc[static_cast<int>(CentroidModel::EEfID::rightFoot)] << 0., 0., 0.5;
    eef_frc[static_cast<int>(CentroidModel::EEfID::leftFoot)] << 0., 0., 0.5;

    iso[static_cast<int>(CentroidModel::EEfID::rightFoot)] =
        robot_->getBodyNodeIsometry(AtlasBodyNode::r_sole);
    iso[static_cast<int>(CentroidModel::EEfID::leftFoot)] =
        robot_->getBodyNodeIsometry(AtlasBodyNode::l_sole);
    iso[static_cast<int>(CentroidModel::EEfID::rightHand)] =
        robot_->getBodyNodeIsometry(AtlasBodyNode::r_hand);
    iso[static_cast<int>(CentroidModel::EEfID::leftHand)] =
        robot_->getBodyNodeIsometry(AtlasBodyNode::l_hand);

    for (int i = 0; i < CentroidModel::numEEf; ++i) {
        Eigen::VectorXd tmp_vec = Eigen::VectorXd::Zero(4);
        Eigen::Quaternion<double> tmp_quat =
            Eigen::Quaternion<double>(iso[i].linear());
        tmp_vec << tmp_quat.w(), tmp_quat.x(), tmp_quat.y(), tmp_quat.z();
        quat[i] = tmp_vec;
        for (int j = 0; j < 3; ++j)
            iso[i].translation()[j] += sp_->global_pos_local[j];
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
    if (sp_->num_step_copy == 0) {
        rem_time = ini_dsp_dur_ - state_machine_time_;
    } else {
        rem_time = dsp_dur_ - state_machine_time_;
    }
    double t_so_far(rem_time);
    int n_phase(1);
    while (true) {
        if (t_so_far >= _param->timeHorizon) break;
        if (n_phase % 2 == 1)
            t_so_far += ssp_dur_;
        else
            t_so_far += dsp_dur_;
        ++n_phase;
    }

    int n_con_seq(ceil(n_phase / 2.0) - 1);

    Eigen::VectorXd rf_pos =
        iso[static_cast<int>(CentroidModel::EEfID::rightFoot)].translation();
    Eigen::VectorXd lf_pos =
        iso[static_cast<int>(CentroidModel::EEfID::leftFoot)].translation();
    if (sp_->phase_copy == 0) {
        int n_rf_con_seq(ceil(n_phase / 2.0));
        int n_lf_con_seq(floor(n_phase / 2.0));
        double rf_st_time(0.);
        double lf_st_time(0.);
        double rf_end_time(rem_time);
        double lf_end_time(rem_time + ssp_dur_ + dsp_dur_);
        // update rf contact sequence
        for (int i = 0; i < n_rf_con_seq; ++i) {
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)].push_back(
                Eigen::VectorXd::Zero(10));
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i][0] =
                rf_st_time;
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i][1] =
                rf_end_time;
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i]
                .segment<3>(2) = rf_pos;
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i]
                .segment<4>(5) = Eigen::VectorXd::Zero(4);
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i][9] =
                static_cast<double>(ContactType::FlatContact);
            rf_st_time = rf_end_time + ssp_dur_;
            rf_end_time += ssp_dur_ + dsp_dur_ + ssp_dur_ + dsp_dur_;
            if (rf_st_time > _param->timeHorizon) break;
            rf_pos[0] += footstep_length_[0];
            rf_pos[1] += footstep_length_[1];
        }
        // update lf contact sequence
        for (int i = 0; i < n_lf_con_seq; ++i) {
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)].push_back(
                Eigen::VectorXd::Zero(10));
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i][0] =
                lf_st_time;
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i][1] =
                lf_end_time;
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i]
                .segment<3>(2) = lf_pos;
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i]
                .segment<4>(5) = Eigen::VectorXd::Zero(4);
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i][9] =
                static_cast<double>(ContactType::FlatContact);
            lf_st_time = lf_end_time + ssp_dur_;
            lf_end_time += ssp_dur_ + dsp_dur_ + ssp_dur_ + dsp_dur_;
            if (lf_st_time > _param->timeHorizon) break;
            lf_pos[0] += footstep_length_[0];
            lf_pos[1] += footstep_length_[1];
        }
    } else {
        int n_rf_con_seq(floor(n_phase / 2.0));
        int n_lf_con_seq(ceil(n_phase / 2.0));
        double rf_st_time(0.);
        double lf_st_time(0.);
        double lf_end_time(rem_time);
        double rf_end_time(rem_time + ssp_dur_ + dsp_dur_);
        // update lf contact sequence
        for (int i = 0; i < n_lf_con_seq; ++i) {
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)].push_back(
                Eigen::VectorXd::Zero(10));
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i][0] =
                lf_st_time;
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i][1] =
                lf_end_time;
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i]
                .segment<3>(2) = lf_pos;
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i]
                .segment<4>(5) = Eigen::VectorXd::Zero(4);
            c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i][9] =
                static_cast<double>(ContactType::FlatContact);
            lf_st_time += ssp_dur_;
            lf_end_time += ssp_dur_ + dsp_dur_ + ssp_dur_ + dsp_dur_;
            if (lf_st_time > _param->timeHorizon) break;
            lf_pos[0] += footstep_length_[0];
            lf_pos[1] += footstep_length_[1];
        }
        // update rf contact sequence
        for (int i = 0; i < n_rf_con_seq; ++i) {
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)].push_back(
                Eigen::VectorXd::Zero(10));
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i][0] =
                rf_st_time;
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i][1] =
                rf_end_time;
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i]
                .segment<3>(2) = rf_pos;
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i]
                .segment<4>(5) = Eigen::VectorXd::Zero(4);
            c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i][9] =
                static_cast<double>(ContactType::FlatContact);
            rf_st_time += ssp_dur_;
            rf_end_time += ssp_dur_ + dsp_dur_ + ssp_dur_ + dsp_dur_;
            if (rf_st_time > _param->timeHorizon) break;
            rf_pos[0] += footstep_length_[0];
            rf_pos[1] += footstep_length_[1];
        }
    }

    _param->UpdateContactPlanInterface(c_seq);

    // =========================================================================
    // update goal state
    // =========================================================================
    Eigen::Vector3d com_displacement;
    for (int i = 0; i < 2; ++i)
        com_displacement[i] = (rf_pos[i] > lf_pos[i]) ? rf_pos[i] : lf_pos[i];
    // com_displacement[2] = com_height_ - r[2];
    com_displacement[2] = 0.;
    _param->UpdateTerminalState(com_displacement);
}

void DoubleSupportCtrl::_compute_torque_wblc(Eigen::VectorXd& gamma) {
    // WBLC
    wblc_->updateSetting(A_, Ainv_, coriolis_, grav_);
    Eigen::VectorXd des_jacc_cmd =
        des_jacc_ +
        Kp_.cwiseProduct(des_jpos_ -
                         sp_->q.segment(Atlas::n_vdof, Atlas::n_adof)) +
        Kd_.cwiseProduct(des_jvel_ - sp_->qdot.tail(Atlas::n_adof));

    wblc_->makeWBLC_Torque(des_jacc_cmd, contact_list_, gamma, wblc_data_);
}

void DoubleSupportCtrl::_task_setup() {
    // =========================================================================
    // Joint Pos Task
    // =========================================================================
    Eigen::VectorXd jpos_des = sp_->jpos_ini;
    Eigen::VectorXd jvel_des(Atlas::n_adof);
    jvel_des.setZero();
    Eigen::VectorXd jacc_des(Atlas::n_adof);
    jacc_des.setZero();
    total_joint_task_->updateTask(jpos_des, jvel_des, jacc_des);

    // =========================================================================
    // Task List Update
    // =========================================================================
    task_list_.push_back(total_joint_task_);

    // =========================================================================
    // Solve Inv Kinematics
    // =========================================================================
    kin_wbc_->FindConfiguration(sp_->q, task_list_, contact_list_, des_jpos_,
                                des_jvel_, des_jacc_);
}

void DoubleSupportCtrl::_contact_setup() {
    rfoot_contact_->updateContactSpec();
    lfoot_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);
}

void DoubleSupportCtrl::firstVisit() {
    jpos_ini_ = sp_->q.segment(Atlas::n_vdof, Atlas::n_adof);
    ctrl_start_time_ = sp_->curr_time;
}

void DoubleSupportCtrl::lastVisit() {}

bool DoubleSupportCtrl::endOfPhase() {
    if (sp_->num_step_copy == 0) {
        if (state_machine_time_ > ini_dsp_dur_) {
            return true;
        }
    } else {
        if (state_machine_time_ > dsp_dur_) {
            return true;
        }
    }

    return false;
}

void DoubleSupportCtrl::ctrlInitialization(const YAML::Node& node) {
    jpos_ini_ = sp_->q.segment(Atlas::n_vdof, Atlas::n_adof);
    try {
        myUtils::readParameter(node, "kp", Kp_);
        myUtils::readParameter(node, "kd", Kd_);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}
