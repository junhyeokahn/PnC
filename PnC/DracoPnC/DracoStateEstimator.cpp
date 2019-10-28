#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/DracoStateEstimator.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/StateEstimator/BasicAccumulation.hpp>
#include <PnC/DracoPnC/StateEstimator/BodyEstimator.hpp>
#include <PnC/DracoPnC/StateEstimator/IMUFrameEstimator.hpp>
#include <PnC/Filters/Basic/filters.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <Utils/IO/IOUtilities.hpp>

DracoStateEstimator::DracoStateEstimator(RobotSystem* robot) {
    myUtils::pretty_constructor(1, "Atlas State Estimator");

    robot_ = robot;
    sp_ = DracoStateProvider::getStateProvider(robot_);
    curr_config_ = Eigen::VectorXd::Zero(robot_->getNumDofs());
    curr_qdot_ = Eigen::VectorXd::Zero(robot_->getNumDofs());
    global_body_euler_zyx_.setZero();
    global_body_quat_ = Eigen::Quaternion<double>::Identity();
    global_body_euler_zyx_dot_.setZero();
    virtual_q_ = Eigen::VectorXd::Zero(6);
    virtual_qdot_ = Eigen::VectorXd::Zero(6);

    ori_est_ = new BasicAccumulation();
    mocap_x_vel_est_ = new AverageFilter(DracoAux::ServoRate, 0.01, 1.0);
    mocap_y_vel_est_ = new AverageFilter(DracoAux::ServoRate, 0.01, 1.5);
    x_vel_est_ = new AverageFilter(DracoAux::ServoRate, 0.01, 1.0);
    y_vel_est_ = new AverageFilter(DracoAux::ServoRate, 0.01, 1.5);
    body_est_ = new BodyEstimator(robot);
    //imu_frame_est_ = new IMUFrameEstimator(robot);
}

DracoStateEstimator::~DracoStateEstimator() {
    delete ori_est_;
    delete body_est_;
    delete mocap_x_vel_est_;
    delete mocap_y_vel_est_;
    delete x_vel_est_;
    delete y_vel_est_;
    //delete imu_frame_est_;
}

void DracoStateEstimator::initialization(DracoSensorData* data) {
    _JointUpdate(data);

    std::vector<double> torso_acc(3);
    std::vector<double> torso_ang_vel(3);
    Eigen::VectorXd torso_acc_2 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd torso_ang_vel_2 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd torso_mag_2 = Eigen::VectorXd::Zero(3);
    MapToTorso_(data->imu_acc, data->imu_ang_vel, torso_acc, torso_ang_vel);
    MapToTorso2_(data->imu_acc, data->imu_ang_vel, data->imu_mag, torso_acc_2,
                 torso_ang_vel_2, torso_mag_2);

    ori_est_->estimatorInitialization(torso_acc, torso_ang_vel);
    ori_est_->getEstimatedState(global_body_euler_zyx_,
                                global_body_euler_zyx_dot_);
    //imu_frame_est_->estimatorInitialization(torso_acc_2, torso_ang_vel_2,
                                            //torso_mag_2);
    //imu_frame_est_->getVirtualJointPosAndVel(virtual_q_, virtual_qdot_);
    global_body_quat_ = Eigen::Quaternion<double>(
        dart::math::eulerZYXToMatrix(global_body_euler_zyx_));
    body_est_->Initialization(global_body_quat_);

    _ConfigurationAndModelUpdate();
    sp_->com_pos = robot_->getCoMPosition();
    sp_->com_vel = robot_->getCoMVelocity();

    ((AverageFilter*)x_vel_est_)->initialization(sp_->com_vel[0]);
    ((AverageFilter*)y_vel_est_)->initialization(sp_->com_vel[1]);
    ((AverageFilter*)mocap_x_vel_est_)->initialization(sp_->com_vel[0]);
    ((AverageFilter*)mocap_y_vel_est_)->initialization(sp_->com_vel[1]);

    _FootContactUpdate(data);

    sp_->saveCurrentData();
}

void DracoStateEstimator::MapToTorso_(const Eigen::VectorXd& imu_acc,
                                      const Eigen::VectorXd& imu_angvel,
                                      std::vector<double>& torso_acc,
                                      std::vector<double>& torso_angvel) {
    Eigen::MatrixXd R_world_imu = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd R_world_torso = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd R_torso_imu = Eigen::MatrixXd::Zero(3, 3);

    Eigen::VectorXd t_acc_local = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd t_angvel_local = Eigen::VectorXd::Zero(3);

    R_world_imu = robot_->getBodyNodeIsometry(DracoBodyNode::IMU).linear();
    R_world_torso = robot_->getBodyNodeIsometry(DracoBodyNode::Torso).linear();
    R_torso_imu = R_world_torso.transpose() * R_world_imu;

    t_acc_local = R_torso_imu * imu_acc;
    t_angvel_local = R_torso_imu * imu_angvel;

    for (int i = 0; i < 3; ++i) {
        torso_acc[i] = t_acc_local[i];
        torso_angvel[i] = t_angvel_local[i];
    }
}

void DracoStateEstimator::MapToTorso2_(const Eigen::VectorXd& imu_acc,
                                       const Eigen::VectorXd& imu_angvel,
                                       const Eigen::VectorXd& imu_mag,
                                       Eigen::VectorXd& torso_acc,
                                       Eigen::VectorXd& torso_ang_vel,
                                       Eigen::VectorXd& torso_mag) {
    Eigen::MatrixXd R_world_imu = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd R_world_torso = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd R_torso_imu = Eigen::MatrixXd::Zero(3, 3);

    Eigen::VectorXd t_acc_local = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd t_angvel_local = Eigen::VectorXd::Zero(3);

    R_world_imu = robot_->getBodyNodeIsometry(DracoBodyNode::IMU).linear();
    R_world_torso = robot_->getBodyNodeIsometry(DracoBodyNode::Torso).linear();
    R_torso_imu = R_world_torso.transpose() * R_world_imu;

    torso_acc = R_torso_imu * imu_acc;
    torso_ang_vel = R_torso_imu * imu_angvel;
    torso_mag = R_torso_imu * imu_mag;
}

void DracoStateEstimator::update(DracoSensorData* data) {
    _JointUpdate(data);

    std::vector<double> torso_acc(3);
    std::vector<double> torso_ang_vel(3);
    Eigen::VectorXd torso_acc_2 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd torso_ang_vel_2 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd torso_mag_2 = Eigen::VectorXd::Zero(3);

    MapToTorso_(data->imu_acc, data->imu_ang_vel, torso_acc, torso_ang_vel);
    MapToTorso2_(data->imu_acc, data->imu_ang_vel, data->imu_mag, torso_acc_2,
                 torso_ang_vel_2, torso_mag_2);

    ori_est_->setSensorData(torso_acc, torso_ang_vel);
    ori_est_->getEstimatedState(global_body_euler_zyx_,
                                global_body_euler_zyx_dot_);
    //imu_frame_est_->estimatorInitialization(torso_acc_2, torso_ang_vel_2,
                                            //torso_mag_2);
    //imu_frame_est_->getVirtualJointPosAndVel(virtual_q_, virtual_qdot_);

    global_body_quat_ = Eigen::Quaternion<double>(
        dart::math::eulerZYXToMatrix(global_body_euler_zyx_));

    _ConfigurationAndModelUpdate();
    sp_->com_pos = robot_->getCoMPosition();
    sp_->com_vel = robot_->getCoMVelocity();

    static bool visit_once(false);
    if ((sp_->phase_copy == 2) && (!visit_once)) {
        body_est_->Initialization(global_body_quat_);
        ((AverageFilter*)x_vel_est_)->initialization(sp_->com_vel[0]);
        ((AverageFilter*)y_vel_est_)->initialization(sp_->com_vel[1]);
        ((AverageFilter*)mocap_x_vel_est_)->initialization(sp_->com_vel[0]);
        ((AverageFilter*)mocap_y_vel_est_)->initialization(sp_->com_vel[1]);
        visit_once = true;
    }

    x_vel_est_->input(sp_->com_vel[0]);
    sp_->est_com_vel[0] = x_vel_est_->output();
    y_vel_est_->input(sp_->com_vel[1]);
    sp_->est_com_vel[1] = y_vel_est_->output();

    Eigen::Vector3d mocap_body_vel;
    body_est_->Update();
    body_est_->getMoCapBodyVel(mocap_body_vel);
    body_est_->getMoCapBodyPos(global_body_quat_, sp_->est_mocap_body_pos);
    mocap_x_vel_est_->input(mocap_body_vel[0]);
    sp_->est_mocap_body_vel[0] = mocap_x_vel_est_->output();
    mocap_y_vel_est_->input(mocap_body_vel[1]);
    sp_->est_mocap_body_vel[1] = mocap_y_vel_est_->output();

    _FootContactUpdate(data);

    sp_->saveCurrentData();
}

void DracoStateEstimator::_JointUpdate(DracoSensorData* data) {
    curr_config_.setZero();
    curr_qdot_.setZero();
    for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
        curr_config_[robot_->getNumVirtualDofs() + i] = data->q[i];
        curr_qdot_[robot_->getNumVirtualDofs() + i] = data->qdot[i];
    }
    sp_->rotor_inertia = data->rotor_inertia;
}

void DracoStateEstimator::_ConfigurationAndModelUpdate() {
    curr_config_[3] = global_body_euler_zyx_[0];
    curr_config_[4] = global_body_euler_zyx_[1];
    curr_config_[5] = global_body_euler_zyx_[2];

    for (int i(0); i < 3; ++i)
        curr_qdot_[i + 3] = global_body_euler_zyx_dot_[i];

    robot_->updateSystem(curr_config_, curr_qdot_, false);
    Eigen::VectorXd foot_pos;
    Eigen::VectorXd foot_vel;
    if (sp_->stance_foot == "rFoot") {
        foot_pos = robot_->getBodyNodeIsometry(DracoBodyNode::rFootCenter)
                       .translation();
        foot_vel =
            robot_->getBodyNodeSpatialVelocity(DracoBodyNode::rFootCenter)
                .tail(3);
    } else {
        foot_pos = robot_->getBodyNodeIsometry(DracoBodyNode::lFootCenter)
                       .translation();
        foot_vel =
            robot_->getBodyNodeSpatialVelocity(DracoBodyNode::lFootCenter)
                .tail(3);
    }

    curr_config_[0] = -foot_pos[0];
    curr_config_[1] = -foot_pos[1];
    curr_config_[2] = -foot_pos[2];
    curr_qdot_[0] = -foot_vel[0];
    curr_qdot_[1] = -foot_vel[1];
    curr_qdot_[2] = -foot_vel[2];

    robot_->updateSystem(curr_config_, curr_qdot_, false);

    sp_->q = curr_config_;
    sp_->qdot = curr_qdot_;

}

void DracoStateEstimator::_FootContactUpdate(DracoSensorData* data) {
    if (data->rfoot_contact)
        sp_->b_rfoot_contact = 1;
    else
        sp_->b_rfoot_contact = 0;
    if (data->lfoot_contact)
        sp_->b_lfoot_contact = 1;
    else
        sp_->b_lfoot_contact = 0;
}
