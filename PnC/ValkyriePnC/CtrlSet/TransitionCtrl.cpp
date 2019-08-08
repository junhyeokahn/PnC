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

TransitionCtrl::TransitionCtrl(RobotSystem* robot, int moving_foot,
                               bool b_increase)
    : Controller(robot), moving_foot_(moving_foot), b_increase_(b_increase) {
    myUtils::pretty_constructor(2, "Transition Ctrl");

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

    centroid_task_ = new BasicTask(robot, BasicTaskType::CENTROID, 6);
    total_joint_task_ =
        new BasicTask(robot, BasicTaskType::JOINT, Valkyrie::n_adof);

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
}

TransitionCtrl::~TransitionCtrl() {
    delete centroid_task_;
    delete total_joint_task_;

    delete kin_wbc_;
    delete wblc_;
    delete wblc_data_;

    delete rfoot_contact_;
    delete lfoot_contact_;
}

void TransitionCtrl::oneStep(void* _cmd) {
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

void TransitionCtrl::_compute_torque_wblc(Eigen::VectorXd& gamma) {
    // WBLC
    wblc_->updateSetting(A_, Ainv_, coriolis_, grav_);
    Eigen::VectorXd des_jacc_cmd =
        des_jacc_ +
        Kp_.cwiseProduct(des_jpos_ -
                         sp_->q.segment(Valkyrie::n_vdof, Valkyrie::n_adof)) +
        Kd_.cwiseProduct(des_jvel_ - sp_->qdot.tail(Valkyrie::n_adof));

    wblc_->makeWBLC_Torque(des_jacc_cmd, contact_list_, gamma, wblc_data_);
}

void TransitionCtrl::_task_setup() {
    // =========================================================================
    // Centroid Task
    // =========================================================================
    double pos[3];
    double vel[3];
    double acc[3];

    com_traj_.getCurvePoint(state_machine_time_, pos);
    com_traj_.getCurveDerPoint(state_machine_time_, 1, vel);
    com_traj_.getCurveDerPoint(state_machine_time_, 2, acc);

    Eigen::VectorXd cen_pos_des = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd cen_vel_des = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd cen_acc_des = Eigen::VectorXd::Zero(6);
    for (int i = 0; i < 3; ++i) {
        cen_pos_des[i + 3] = pos[i];
        cen_vel_des[i] = 0.;
        cen_vel_des[i + 3] = vel[i] * robot_->getRobotMass();
    }

    for (int i = 0; i < 3; ++i) {
        sp_->com_pos_des[i] = cen_pos_des[i + 3];
        sp_->com_vel_des[i] = cen_vel_des[i + 3] / robot_->getRobotMass();
        sp_->mom_des[i] = cen_vel_des[i];
        sp_->mom_des[i + 3] = cen_vel_des[i + 3];
    }

    centroid_task_->updateTask(cen_pos_des, cen_vel_des, cen_acc_des);

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
    task_list_.push_back(centroid_task_);
    task_list_.push_back(total_joint_task_);

    // =========================================================================
    // Solve Inv Kinematics
    // =========================================================================
    kin_wbc_->FindConfiguration(sp_->q, task_list_, contact_list_, des_jpos_,
                                des_jvel_, des_jacc_);
}

void TransitionCtrl::_contact_setup() {
    double alpha = 0.5 * (1 - cos(M_PI * state_machine_time_ / trns_dur_));
    double upper_lim(0.);
    double rf_weight(0.);
    double rf_weight_z(0.);
    double foot_weight(0.);

    // set swing foot's weight and limit
    if (b_increase_) {
        upper_lim = min_rf_z_ + alpha * (max_rf_z_ - min_rf_z_);  // min->max
        rf_weight = (1. - alpha) * 5. + alpha * 1.0;              // 5->1
        rf_weight_z = (1. - alpha) * 0.5 + alpha * 0.01;          // 0.5->0.01
        foot_weight = 0.001 * (1. - alpha) + 1000. * alpha;  // 0.001 -> 1000
    } else {
        upper_lim = max_rf_z_ - alpha * (max_rf_z_ - min_rf_z_);  // max->min
        rf_weight = (alpha)*5. + (1. - alpha) * 1.0;              // 1->5
        rf_weight_z = (alpha)*0.5 + (1. - alpha) * 0.01;          // 0.01->0.5
        foot_weight = 0.001 * (alpha) + 1000. * (1. - alpha);  // 1000 -> 0.001
    }
    rfoot_contact_->updateContactSpec();
    lfoot_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);

    int jidx_offset(0);
    if (moving_foot_ == ValkyrieBodyNode::leftFoot) {
        jidx_offset = rfoot_contact_->getDim();
        for (int i = 0; i < lfoot_contact_->getDim(); ++i) {
            wblc_data_->W_rf_[i + jidx_offset] = rf_weight;
            wblc_data_->W_xddot_[i + jidx_offset] = foot_weight;
        }
        wblc_data_->W_rf_[lfoot_contact_->getFzIndex() + jidx_offset] =
            rf_weight_z;

        ((SurfaceContactSpec*)lfoot_contact_)->setMaxFz(upper_lim);
    } else if (moving_foot_ == ValkyrieBodyNode::rightFoot) {
        for (int i(0); i < rfoot_contact_->getDim(); ++i) {
            wblc_data_->W_rf_[i + jidx_offset] = rf_weight;
            wblc_data_->W_xddot_[i + jidx_offset] = foot_weight;
        }
        wblc_data_->W_rf_[rfoot_contact_->getFzIndex() + jidx_offset] =
            rf_weight_z;

        ((SurfaceContactSpec*)rfoot_contact_)->setMaxFz(upper_lim);
    }
}

void TransitionCtrl::firstVisit() {
    jpos_ini_ = sp_->q.segment(Valkyrie::n_vdof, Valkyrie::n_adof);
    ctrl_start_time_ = sp_->curr_time;
    SetBSpline_();
}

void TransitionCtrl::SetBSpline_() {
    double ini[9];
    double fin[9];
    double** middle_pt = new double*[1];
    middle_pt[0] = new double[3];

    Eigen::Vector3d ini_pos = robot_->getCoMPosition();
    Eigen::Vector3d ini_vel = robot_->getCoMVelocity();
    Eigen::VectorXd fin_pos = Eigen::VectorXd::Zero(3);

    if (b_increase_) {
        fin_pos = robot_->getBodyNodeIsometry(moving_cop_).translation();
    } else {
        fin_pos = robot_->getBodyNodeIsometry(stance_cop_).translation();
    }
    fin_pos[2] = ini_pos[2];

    for (int i = 0; i < 3; ++i) {
        ini[i] = ini_pos[i];
        ini[i + 3] = ini_vel[i];
        ini[i + 6] = 0.;

        fin[i] = fin_pos[i];
        fin[i + 3] = 0.;
        fin[i + 6] = 0.;

        middle_pt[0][i] = (ini_pos[i] + fin_pos[i]) / 2.;
    }
    com_traj_.SetParam(ini, fin, middle_pt, trns_dur_);

    delete[] * middle_pt;
    delete[] middle_pt;
}

void TransitionCtrl::lastVisit() {}

bool TransitionCtrl::endOfPhase() {
    if (state_machine_time_ > trns_dur_) {
        return true;
    }
    return false;
}

void TransitionCtrl::ctrlInitialization(const YAML::Node& node) {
    jpos_ini_ = sp_->q.segment(Valkyrie::n_vdof, Valkyrie::n_adof);
    try {
        myUtils::readParameter(node, "kp", Kp_);
        myUtils::readParameter(node, "kd", Kd_);
        myUtils::readParameter(node, "max_rf_z", max_rf_z_);
        myUtils::readParameter(node, "min_rf_z", min_rf_z_);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}
