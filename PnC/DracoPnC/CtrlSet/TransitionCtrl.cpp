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

TransitionCtrl::TransitionCtrl(RobotSystem* robot, Planner* planner,
                               int moving_foot, bool b_increase)
    : Controller(robot),
      planner_(planner),
      moving_foot_(moving_foot),
      b_increase_(b_increase) {
    myUtils::pretty_constructor(2, "Transition Ctrl");

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
    rfoot_contact_ = new SurfaceContactSpec(
        robot_, DracoBodyNode::rFootCenter, 0.085, 0.02, 0.7);
    lfoot_contact_ = new SurfaceContactSpec(
        robot_, DracoBodyNode::lFootCenter, 0.085, 0.02, 0.7);
    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();

    wblc_data_->W_qddot_ = Eigen::VectorXd::Constant(Draco::n_dof, 100.0);
    wblc_data_->W_rf_ = Eigen::VectorXd::Constant(dim_contact_, 0.1);
    wblc_data_->W_xddot_ = Eigen::VectorXd::Constant(dim_contact_, 1000.0);
    wblc_data_->W_rf_[rfoot_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_contact_->getDim() + lfoot_contact_->getFzIndex()] =
        0.01;

    // torque limit default setting
    wblc_data_->tau_min_ = Eigen::VectorXd::Constant(Draco::n_adof, -2500.);
    wblc_data_->tau_max_ = Eigen::VectorXd::Constant(Draco::n_adof, 2500.);

    sp_ = DracoStateProvider::getStateProvider(robot);
}

TransitionCtrl::~TransitionCtrl() {
    delete com_task_;
    delete total_joint_task_;
    delete torso_ori_task_;

    delete kin_wbc_;
    delete wblc_;
    delete wblc_data_;

    delete rfoot_contact_;
    delete lfoot_contact_;
}

void TransitionCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;

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

void TransitionCtrl::_compute_torque_wblc(Eigen::VectorXd& gamma) {
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

void TransitionCtrl::_task_setup() {
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

    // initial pos
    com_pos_des = ini_com_pos_;
    // centroid planner
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
        rf_weight = (1. - alpha) * 5. + alpha * 0.1;              // 5->0.1
        rf_weight_z = (1. - alpha) * 0.5 + alpha * 0.01;          // 0.5->0.01
        foot_weight = 0.001 * (1. - alpha) + 1000. * alpha;  // 0.001 -> 1000
    } else {
        upper_lim = max_rf_z_ - alpha * (max_rf_z_ - min_rf_z_);  // max->min
        rf_weight = (alpha)*5. + (1. - alpha) * 0.1;              // 0.1->5
        rf_weight_z = (alpha)*0.5 + (1. - alpha) * 0.01;          // 0.01->0.5
        foot_weight = 0.001 * (alpha) + 1000. * (1. - alpha);  // 1000 -> 0.001
    }
    rfoot_contact_->updateContactSpec();
    lfoot_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);

    int jidx_offset(0);
    if (moving_foot_ == DracoBodyNode::lAnkle) {
        jidx_offset = rfoot_contact_->getDim();
        for (int i = 0; i < lfoot_contact_->getDim(); ++i) {
            wblc_data_->W_rf_[i + jidx_offset] = rf_weight;
            wblc_data_->W_xddot_[i + jidx_offset] = foot_weight;
        }
        wblc_data_->W_rf_[lfoot_contact_->getFzIndex() + jidx_offset] =
            rf_weight_z;

        ((SurfaceContactSpec*)lfoot_contact_)->setMaxFz(upper_lim);
    } else if (moving_foot_ == DracoBodyNode::rAnkle) {
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
    jpos_ini_ = sp_->q.segment(Draco::n_vdof, Draco::n_adof);
    ini_com_pos_ = Eigen::VectorXd::Zero(3);
    ini_com_pos_ << robot_->getCoMPosition()[0], robot_->getCoMPosition()[1],
        robot_->getCoMPosition()[2];
    ctrl_start_time_ = sp_->curr_time;
}

void TransitionCtrl::lastVisit() {}

bool TransitionCtrl::endOfPhase() {
    if (state_machine_time_ > trns_dur_) {
        return true;
    }
    return false;
}

void TransitionCtrl::ctrlInitialization(const YAML::Node& node) {
    jpos_ini_ = sp_->q.segment(Draco::n_vdof, Draco::n_adof);
    try {
        myUtils::readParameter(node, "kp", Kp_);
        myUtils::readParameter(node, "kd", Kd_);
        myUtils::readParameter(node, "max_rf_z", max_rf_z_);
        myUtils::readParameter(node, "min_rf_z", min_rf_z_);

        Eigen::VectorXd tmp_vec1, tmp_vec2;
        myUtils::readParameter(node, "com_kp", tmp_vec1);
        myUtils::readParameter(node, "com_kd", tmp_vec2);
        com_task_->setGain(tmp_vec1, tmp_vec2);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}
