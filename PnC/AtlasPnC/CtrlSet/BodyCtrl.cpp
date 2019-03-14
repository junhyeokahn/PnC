#include <PnC/AtlasPnC/AtlasDefinition.hpp>
#include <PnC/AtlasPnC/AtlasInterface.hpp>
#include <PnC/AtlasPnC/AtlasStateProvider.hpp>
#include <PnC/AtlasPnC/ContactSet/ContactSet.hpp>
#include <PnC/AtlasPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/AtlasPnC/TaskSet/TaskSet.hpp>
#include <PnC/WBC/WBLC/KinWBC.hpp>
#include <PnC/WBC/WBLC/WBLC.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/Math/MathUtilities.hpp>

BodyCtrl::BodyCtrl(RobotSystem* robot) : Controller(robot) {
    myUtils::pretty_constructor(2, "Body Ctrl");
    end_time_ = 1000.;
    ctrl_start_time_ = 0.;
    b_set_height_target_ = false;
    des_jpos_ = Eigen::VectorXd::Zero(Atlas::n_adof);
    des_jvel_ = Eigen::VectorXd::Zero(Atlas::n_adof);
    des_jacc_ = Eigen::VectorXd::Zero(Atlas::n_adof);
    Kp_ = Eigen::VectorXd::Zero(Atlas::n_adof);
    Kd_ = Eigen::VectorXd::Zero(Atlas::n_adof);

    total_joint_task_ =
        new BasicTask(robot, BasicTaskType::JOINT, Atlas::n_adof);
    body_pos_task_ =
        new BasicTask(robot, BasicTaskType::LINKXYZ, 3, AtlasBodyNode::pelvis);
    body_ori_task_ =
        new BasicTask(robot, BasicTaskType::LINKORI, 3, AtlasBodyNode::pelvis);
    torso_ori_task_ =
        new BasicTask(robot, BasicTaskType::LINKORI, 3, AtlasBodyNode::utorso);

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

BodyCtrl::~BodyCtrl() {
    delete total_joint_task_;
    delete wblc_;
    delete wblc_data_;
    delete rfoot_contact_;
    delete lfoot_contact_;
}

void BodyCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;

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

void BodyCtrl::_compute_torque_wblc(Eigen::VectorXd& gamma) {
    // WBLC
    wblc_->updateSetting(A_, Ainv_, coriolis_, grav_);
    Eigen::VectorXd des_jacc_cmd =
        des_jacc_ +
        Kp_.cwiseProduct(des_jpos_ -
                         sp_->q.segment(Atlas::n_vdof, Atlas::n_adof)) +
        Kd_.cwiseProduct(des_jvel_ - sp_->qdot.tail(Atlas::n_adof));

    wblc_->makeWBLC_Torque(des_jacc_cmd, contact_list_, gamma, wblc_data_);
}

void BodyCtrl::_task_setup() {
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
    Eigen::VectorXd jvel_des(Atlas::n_adof);
    jvel_des.setZero();
    Eigen::VectorXd jacc_des(Atlas::n_adof);
    jacc_des.setZero();
    total_joint_task_->updateTask(jpos_des, jvel_des, jacc_des);

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
}

void BodyCtrl::_contact_setup() {
    rfoot_contact_->updateContactSpec();
    lfoot_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);
}

void BodyCtrl::firstVisit() {
    jpos_ini_ = sp_->q.segment(Atlas::n_vdof, Atlas::n_adof);
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

void BodyCtrl::lastVisit() {}

bool BodyCtrl::endOfPhase() {
    if (state_machine_time_ > end_time_) {
        return true;
    }
    return false;
}

void BodyCtrl::ctrlInitialization(const YAML::Node& node) {
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
