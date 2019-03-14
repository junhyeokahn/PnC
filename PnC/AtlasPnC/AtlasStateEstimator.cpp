#include <PnC/AtlasPnC/AtlasDefinition.hpp>
#include <PnC/AtlasPnC/AtlasInterface.hpp>
#include <PnC/AtlasPnC/AtlasStateEstimator.hpp>
#include <PnC/AtlasPnC/AtlasStateProvider.hpp>
#include <PnC/AtlasPnC/StateEstimator/BasicAccumulation.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <Utils/IO/IOUtilities.hpp>

AtlasStateEstimator::AtlasStateEstimator(RobotSystem* robot) {
    myUtils::pretty_constructor(1, "Atlas State Estimator");

    robot_ = robot;
    sp_ = AtlasStateProvider::getStateProvider(robot_);
    curr_config_ = Eigen::VectorXd::Zero(Atlas::n_dof);
    curr_qdot_ = Eigen::VectorXd::Zero(Atlas::n_dof);
    global_body_euler_zyx_.setZero();
    global_body_quat_ = Eigen::Quaternion<double>::Identity();
    global_body_euler_zyx_dot_.setZero();
    ori_est_ = new BasicAccumulation();
}

AtlasStateEstimator::~AtlasStateEstimator() { delete ori_est_; }

void AtlasStateEstimator::Initialization(AtlasSensorData* data) {
    _JointUpdate(data);

    std::vector<double> imu_acc(3);
    std::vector<double> imu_ang_vel(3);
    for (int i(0); i < 3; ++i) {
        imu_acc[i] = data->imu_acc[i];
        imu_ang_vel[i] = data->imu_ang_vel[i];
    }
    ori_est_->estimatorInitialization(imu_acc, imu_ang_vel);
    ori_est_->getEstimatedState(global_body_euler_zyx_,
                                global_body_euler_zyx_dot_);
    global_body_quat_ = Eigen::Quaternion<double>(
        dart::math::eulerZYXToMatrix(global_body_euler_zyx_));

    _ConfigurationAndModelUpdate();

    _FootContactUpdate(data);

    sp_->saveCurrentData();
}

void AtlasStateEstimator::Update(AtlasSensorData* data) {
    _JointUpdate(data);

    std::vector<double> imu_acc(3);
    std::vector<double> imu_ang_vel(3);
    for (int i(0); i < 3; ++i) {
        imu_acc[i] = data->imu_acc[i];
        imu_ang_vel[i] = data->imu_ang_vel[i];
    }
    ori_est_->setSensorData(imu_acc, imu_ang_vel);
    ori_est_->getEstimatedState(global_body_euler_zyx_,
                                global_body_euler_zyx_dot_);
    global_body_quat_ = Eigen::Quaternion<double>(
        dart::math::eulerZYXToMatrix(global_body_euler_zyx_));

    _ConfigurationAndModelUpdate();

    _FootContactUpdate(data);

    sp_->saveCurrentData();
}

void AtlasStateEstimator::_JointUpdate(AtlasSensorData* data) {
    curr_config_.setZero();
    curr_qdot_.setZero();
    for (int i(0); i < Atlas::n_adof; ++i) {
        curr_config_[Atlas::n_vdof + i] = data->q[i];
        curr_qdot_[Atlas::n_vdof + i] = data->qdot[i];
    }
}

void AtlasStateEstimator::_ConfigurationAndModelUpdate() {
    curr_config_[3] = global_body_euler_zyx_[0];
    curr_config_[4] = global_body_euler_zyx_[1];
    curr_config_[5] = global_body_euler_zyx_[2];

    for (int i(0); i < 3; ++i)
        curr_qdot_[i + 3] = global_body_euler_zyx_dot_[i];

    robot_->updateSystem(curr_config_, curr_qdot_, false);
    Eigen::VectorXd foot_pos =
        robot_->getBodyNodeIsometry(sp_->stance_foot).translation();
    Eigen::VectorXd foot_vel =
        robot_->getBodyNodeSpatialVelocity(sp_->stance_foot).tail(3);

    curr_config_[0] = -foot_pos[0];
    curr_config_[1] = -foot_pos[1];
    curr_config_[2] = -foot_pos[2];
    curr_qdot_[0] = -foot_vel[0];
    curr_qdot_[1] = -foot_vel[1];
    curr_qdot_[2] = -foot_vel[2];

    robot_->updateSystem(curr_config_, curr_qdot_, false);

    sp_->q = curr_config_;
    sp_->qdot = curr_qdot_;
    sp_->jpos_ini = curr_config_.segment(Atlas::n_vdof, Atlas::n_adof);
}

void AtlasStateEstimator::_FootContactUpdate(AtlasSensorData* data) {
    if (data->rfoot_contact)
        sp_->b_rfoot_contact = 1;
    else
        sp_->b_rfoot_contact = 0;
    if (data->lfoot_contact)
        sp_->b_lfoot_contact = 1;
    else
        sp_->b_lfoot_contact = 0;
}
