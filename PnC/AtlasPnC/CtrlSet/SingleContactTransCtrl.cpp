#include <Configuration.h>
#include <PnC/AtlasPnC/AtlasDefinition.hpp>
#include <PnC/AtlasPnC/AtlasInterface.hpp>
#include <PnC/AtlasPnC/AtlasStateProvider.hpp>
#include <PnC/AtlasPnC/ContactSet/ContactSet.hpp>
#include <PnC/AtlasPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/AtlasPnC/TaskSet/TaskSet.hpp>
#include <PnC/WBC/WBLC/KinWBC.hpp>
#include <PnC/WBC/WBLC/WBLC.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

SingleContactTransCtrl::SingleContactTransCtrl(RobotSystem* robot,
                                               int moving_foot, bool b_increase)
    : Controller(robot),
      b_set_height_target_(false),
      moving_foot_(moving_foot),
      b_increase_(b_increase),
      end_time_(100.),
      ctrl_start_time_(0.),
      des_jpos_(Atlas::n_adof),
      des_jvel_(Atlas::n_adof),
      des_jacc_(Atlas::n_adof),
      Kp_(Atlas::n_adof),
      Kd_(Atlas::n_adof) {
    myUtils::pretty_constructor(2, "Single Contact Trans Ctrl");
    total_joint_task_ =
        new BasicTask(robot_, BasicTaskType::JOINT, Atlas::n_adof);
    body_pos_task_ =
        new BasicTask(robot_, BasicTaskType::LINKXYZ, 3, AtlasBodyNode::pelvis);
    body_ori_task_ =
        new BasicTask(robot_, BasicTaskType::LINKORI, 3, AtlasBodyNode::pelvis);
    torso_ori_task_ =
        new BasicTask(robot_, BasicTaskType::LINKORI, 3, AtlasBodyNode::utorso);

    rfoot_contact_ = new SurfaceContactSpec(robot_, AtlasBodyNode::r_sole,
                                            0.125, 0.075, 0.9);
    lfoot_contact_ = new SurfaceContactSpec(robot_, AtlasBodyNode::l_sole,
                                            0.125, 0.075, 0.9);
    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();

    std::vector<bool> act_list;
    act_list.resize(Atlas::n_dof, true);
    for (int i(0); i < Atlas::n_vdof; ++i) act_list[i] = false;

    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);
    wblc_data_ = new WBLC_ExtraData();
    wblc_data_->W_qddot_ = Eigen::VectorXd::Constant(Atlas::n_dof, 100.0);
    wblc_data_->W_rf_ = Eigen::VectorXd::Constant(dim_contact_, 1.0);
    wblc_data_->W_xddot_ = Eigen::VectorXd::Constant(dim_contact_, 1000.0);
    wblc_data_->W_rf_[rfoot_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_contact_->getDim() + lfoot_contact_->getFzIndex()] =
        0.01;

    // torque limit default setting
    wblc_data_->tau_min_ = Eigen::VectorXd::Constant(Atlas::n_adof, -2500.);
    wblc_data_->tau_max_ = Eigen::VectorXd::Constant(Atlas::n_adof, 2500.);

    sp_ = AtlasStateProvider::getStateProvider(robot_);
}

SingleContactTransCtrl::~SingleContactTransCtrl() {
    delete total_joint_task_;
    delete rfoot_contact_;
    delete lfoot_contact_;

    delete kin_wbc_;
    delete wblc_;
    delete wblc_data_;
}

void SingleContactTransCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;
    Eigen::VectorXd gamma;

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

void SingleContactTransCtrl::_compute_torque_wblc(Eigen::VectorXd& gamma) {
    wblc_->updateSetting(A_, Ainv_, coriolis_, grav_);
    Eigen::VectorXd des_jacc_cmd =
        des_jacc_ +
        Kp_.cwiseProduct(des_jpos_ -
                         sp_->q.segment(Atlas::n_vdof, Atlas::n_adof)) +
        Kd_.cwiseProduct(des_jvel_ - sp_->qdot.tail(Atlas::n_adof));

    wblc_->makeWBLC_Torque(des_jacc_cmd, contact_list_, gamma, wblc_data_);

    // dynacore::pretty_print(des_jpos_, std::cout, "des_jpos");
    // dynacore::pretty_print(des_jvel_, std::cout, "des_jvel");
    // dynacore::pretty_print(des_jacc_, std::cout, "des_jacc");
    // dynacore::pretty_print(des_jacc_cmd, std::cout, "des_jacc");
    // dynacore::pretty_print(gamma, std::cout, "gamma");
}

void SingleContactTransCtrl::_task_setup() {
    double t(myUtils::smooth_changing(0, 1, end_time_, state_machine_time_));
    double tdot(
        myUtils::smooth_changing_vel(0, 1, end_time_, state_machine_time_));

    // =========================================================================
    // Body Pos Task
    // =========================================================================
    double body_height_cmd;
    if (b_set_height_target_)
        body_height_cmd = target_body_height_;
    else
        body_height_cmd = ini_body_pos_[2];

    Eigen::VectorXd vel_des(3);
    vel_des.setZero();
    Eigen::VectorXd acc_des(3);
    acc_des.setZero();
    Eigen::VectorXd des_pos = ini_body_pos_;
    des_pos[2] = body_height_cmd;
    body_pos_task_->updateTask(des_pos, vel_des, acc_des);

    // =========================================================================
    // Body Ori Task
    // =========================================================================
    Eigen::Quaternion<double> curr_body_delta_quat =
        dart::math::expToQuat(body_delta_so3_ * t);
    Eigen::Quaternion<double> curr_body_quat_des =
        curr_body_delta_quat * ini_body_quat_;
    Eigen::Vector3d curr_body_so3_des = body_delta_so3_ * tdot;

    Eigen::VectorXd des_body_quat = Eigen::VectorXd::Zero(4);
    des_body_quat << curr_body_quat_des.w(), curr_body_quat_des.x(),
        curr_body_quat_des.y(), curr_body_quat_des.z();
    Eigen::VectorXd des_body_so3 = Eigen::VectorXd::Zero(3);
    for (int i = 0; i < 3; ++i) des_body_so3[i] = curr_body_so3_des[i];

    Eigen::VectorXd ang_acc_des = Eigen::VectorXd::Zero(3);

    body_ori_task_->updateTask(des_body_quat, des_body_so3, ang_acc_des);

    // =========================================================================
    // Torso Ori Task
    // =========================================================================
    Eigen::Quaternion<double> curr_torso_delta_quat =
        dart::math::expToQuat(torso_delta_so3_ * t);
    Eigen::Quaternion<double> curr_torso_quat_des =
        curr_torso_delta_quat * ini_torso_quat_;
    Eigen::Vector3d curr_torso_so3_des = torso_delta_so3_ * tdot;

    Eigen::VectorXd des_torso_quat = Eigen::VectorXd::Zero(4);
    des_torso_quat << curr_torso_quat_des.w(), curr_torso_quat_des.x(),
        curr_torso_quat_des.y(), curr_torso_quat_des.z();
    Eigen::VectorXd des_torso_so3 = Eigen::VectorXd::Zero(3);
    for (int i = 0; i < 3; ++i) des_torso_so3[i] = curr_torso_so3_des[i];

    torso_ori_task_->updateTask(des_torso_quat, des_torso_so3, ang_acc_des);

    // =========================================================================
    // Joint Pos Task
    // =========================================================================
    Eigen::VectorXd jpos_des = sp_->jpos_ini;
    Eigen::VectorXd zero(Atlas::n_adof);
    zero.setZero();
    total_joint_task_->updateTask(jpos_des, zero, zero);

    // =========================================================================
    // Task List Update
    // =========================================================================
    task_list_.push_back(body_pos_task_);
    task_list_.push_back(body_ori_task_);
    task_list_.push_back(torso_ori_task_);
    task_list_.push_back(total_joint_task_);

    // =========================================================================
    // Solve Inv Kinematics
    // =========================================================================
    kin_wbc_->FindConfiguration(sp_->q, task_list_, contact_list_, des_jpos_,
                                des_jvel_, des_jacc_);

    if (b_increase_) {
        if (moving_foot_ == AtlasBodyNode::r_sole) {
            int swing_jidx = AtlasDoF::r_leg_hpz - Atlas::n_vdof;
            double h(state_machine_time_ / end_time_);

            for (int i(0); i < Atlas::num_leg_joint; ++i) {
                des_jpos_[i + swing_jidx] *= h;
                des_jpos_[i + swing_jidx] +=
                    (1. - h) * sp_->des_jpos_prev[swing_jidx + i];
            }
        } else if (moving_foot_ == AtlasBodyNode::l_sole) {
            int swing_jidx = AtlasDoF::l_leg_hpz - Atlas::n_vdof;
            double h(state_machine_time_ / end_time_);

            for (int i(0); i < Atlas::num_leg_joint; ++i) {
                des_jpos_[i + swing_jidx] *= h;
                des_jpos_[i + swing_jidx] +=
                    (1. - h) * sp_->des_jpos_prev[swing_jidx + i];
            }
        }
    }

    // myUtils::pretty_print(jpos_ini, std::cout, "jpos ini");
    // myUtils::pretty_print(des_jpos_, std::cout, "des jpos");
    // myUtils::pretty_print(des_jvel_, std::cout, "des jvel");
    // myUtils::pretty_print(des_jacc_, std::cout, "des jacc");
}

void SingleContactTransCtrl::_contact_setup() {
    double alpha = 0.5 * (1 - cos(M_PI * state_machine_time_ / end_time_));
    double upper_lim(100.);
    double rf_weight(100.);
    double rf_weight_z(100.);
    double foot_weight(1000.);

    if (b_increase_) {
        upper_lim = min_rf_z_ + alpha * (max_rf_z_ - min_rf_z_);
        rf_weight = (1. - alpha) * 5. + alpha * 1.0;
        rf_weight_z = (1. - alpha) * 0.5 + alpha * 0.01;
        foot_weight = 0.001 * (1. - alpha) + 1000. * alpha;
    } else {
        upper_lim = max_rf_z_ - alpha * (max_rf_z_ - min_rf_z_);
        rf_weight = (alpha)*5. + (1. - alpha) * 1.0;
        rf_weight_z = (alpha)*0.5 + (1. - alpha) * 0.01;
        foot_weight = 0.001 * (alpha) + 1000. * (1. - alpha);
    }
    rfoot_contact_->updateContactSpec();
    lfoot_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);

    int jidx_offset(0);
    if (moving_foot_ == AtlasBodyNode::l_sole) {
        jidx_offset = rfoot_contact_->getDim();
        for (int i(0); i < lfoot_contact_->getDim(); ++i) {
            wblc_data_->W_rf_[i + jidx_offset] = rf_weight;
            wblc_data_->W_xddot_[i + jidx_offset] = foot_weight;
        }
        wblc_data_->W_rf_[lfoot_contact_->getFzIndex() + jidx_offset] =
            rf_weight_z;

        ((SurfaceContactSpec*)lfoot_contact_)->setMaxFz(upper_lim);
    } else if (moving_foot_ == AtlasBodyNode::r_sole) {
        for (int i(0); i < rfoot_contact_->getDim(); ++i) {
            wblc_data_->W_rf_[i + jidx_offset] = rf_weight;
            wblc_data_->W_xddot_[i + jidx_offset] = foot_weight;
        }
        wblc_data_->W_rf_[rfoot_contact_->getFzIndex() + jidx_offset] =
            rf_weight_z;

        ((SurfaceContactSpec*)rfoot_contact_)->setMaxFz(upper_lim);
    }
}

void SingleContactTransCtrl::firstVisit() {
    ctrl_start_time_ = sp_->curr_time;

    ini_body_pos_ =
        robot_->getBodyNodeIsometry(AtlasBodyNode::pelvis).translation();

    ini_body_quat_ = Eigen::Quaternion<double>(
        robot_->getBodyNodeIsometry(AtlasBodyNode::pelvis).linear());
    body_delta_quat_ = sp_->des_quat * ini_body_quat_.inverse();
    body_delta_so3_ = dart::math::quatToExp(body_delta_quat_);

    ini_torso_quat_ = Eigen::Quaternion<double>(
        robot_->getBodyNodeIsometry(AtlasBodyNode::utorso).linear());
    torso_delta_quat_ = sp_->des_quat * ini_torso_quat_.inverse();
    torso_delta_so3_ = dart::math::quatToExp(torso_delta_quat_);
}

void SingleContactTransCtrl::lastVisit() { sp_->des_jpos_prev = des_jpos_; }

bool SingleContactTransCtrl::endOfPhase() {
    if (state_machine_time_ > end_time_) {
        return true;
    }
    return false;
}

void SingleContactTransCtrl::ctrlInitialization(const YAML::Node& node) {
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
