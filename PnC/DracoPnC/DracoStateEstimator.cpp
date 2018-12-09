#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/DracoStateEstimator.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <Utils/Utilities.hpp>
#include <PnC/DracoPnC/StateEstimator/BasicAccumulation.hpp>
#include <PnC/DracoPnC/StateEstimator/BodyEstimator.hpp>
#include <Filter/filters.hpp>
#include <RobotSystem/RobotSystem.hpp>

DracoStateEstimator::DracoStateEstimator(RobotSystem* robot) {
    myUtils::pretty_constructor(1, "State Estimator");

    robot_ = robot;
    sp_ = DracoStateProvider::getStateProvider(robot_);
    curr_config_ = Eigen::VectorXd::Zero(robot_->getNumDofs());
    curr_qdot_= Eigen::VectorXd::Zero(robot_->getNumDofs());
    global_body_euler_zyx_.setZero();
    global_body_quat_ = Eigen::Quaternion<double>::Identity();
    global_body_euler_zyx_dot_.setZero();
    ori_est_ = new BasicAccumulation();

    mocap_x_vel_est_ = new AverageFilter(SERVO_RATE, 0.01, 1.0);
    mocap_y_vel_est_ = new AverageFilter(SERVO_RATE, 0.01, 1.5);
    x_vel_est_ = new AverageFilter(SERVO_RATE, 0.01, 1.0);
    y_vel_est_ = new AverageFilter(SERVO_RATE, 0.01, 1.5);
    body_est_ = new BodyEstimator(robot);
}

DracoStateEstimator::~DracoStateEstimator(){
    delete ori_est_;
    delete body_est_;
    delete mocap_x_vel_est_;
    delete mocap_y_vel_est_;
    delete x_vel_est_;
    delete y_vel_est_;
}

void DracoStateEstimator::initialization(DracoSensorData* data){

    _JointUpdate(data);

    std::vector<double> imu_acc(3);
    std::vector<double> imu_ang_vel(3);
    for(int i(0); i<3; ++i){
        imu_acc[i]  = data->imu_acc[i];
        imu_ang_vel[i] = data->imu_ang_vel[i];
    }
    ori_est_->estimatorInitialization(imu_acc, imu_ang_vel);
    ori_est_->getEstimatedState(global_body_euler_zyx_, global_body_euler_zyx_dot_);
    global_body_quat_ = Eigen::Quaternion<double>(dart::math::eulerZYXToMatrix(global_body_euler_zyx_));
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

void DracoStateEstimator::update(DracoSensorData* data){

    _JointUpdate(data);

    std::vector<double> imu_acc(3);
    std::vector<double> imu_ang_vel(3);
    for(int i(0); i<3; ++i){
        imu_acc[i]  = data->imu_acc[i];
        imu_ang_vel[i] = data->imu_ang_vel[i];
    }
    ori_est_->setSensorData( imu_acc, imu_ang_vel);
    ori_est_->getEstimatedState(global_body_euler_zyx_, global_body_euler_zyx_dot_);
    global_body_quat_ = Eigen::Quaternion<double>(dart::math::eulerZYXToMatrix(global_body_euler_zyx_));

    _ConfigurationAndModelUpdate();
    sp_->com_pos = robot_->getCoMPosition();
    sp_->com_vel = robot_->getCoMVelocity();

    static bool visit_once(false);
    if ((sp_->phase_copy == 2) && (!visit_once)){
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
    for (int i(0); i<robot_->getNumActuatedDofs(); ++i){
        curr_config_[robot_->getNumVirtualDofs() + i] = data->q[i];
        curr_qdot_[robot_->getNumVirtualDofs() + i] = data->qdot[i];
    }
}

void DracoStateEstimator::_ConfigurationAndModelUpdate() {

    curr_config_[3] = global_body_euler_zyx_[0];
    curr_config_[4] = global_body_euler_zyx_[1];
    curr_config_[5] = global_body_euler_zyx_[2];

    for(int i(0); i<3; ++i)  curr_qdot_[i+3] = global_body_euler_zyx_dot_[i];

    robot_->updateSystem(curr_config_, curr_qdot_, false);
    Eigen::VectorXd foot_pos;
    Eigen::VectorXd foot_vel;
    if (sp_->stance_foot == "rFoot") {
        foot_pos = robot_->getBodyNodeIsometry("rFootCenter").translation();
        foot_vel = robot_->getBodyNodeSpatialVelocity("rFootCenter").tail(3);
    } else {
        foot_pos = robot_->getBodyNodeIsometry("lFootCenter").translation();
        foot_vel = robot_->getBodyNodeSpatialVelocity("lFootCenter").tail(3);
    }

    curr_config_[0] = -foot_pos[0];
    curr_config_[1] = -foot_pos[1];
    curr_config_[2] = -foot_pos[2];
    curr_qdot_[0] = -foot_vel[0];
    curr_qdot_[1] = -foot_vel[1];
    curr_qdot_[2] = -foot_vel[2];

    //robot_->updateSystem(curr_config_, curr_qdot_, true);
    robot_->updateSystem(curr_config_, curr_qdot_, false);

    sp_->q = curr_config_;
    sp_->qdot = curr_qdot_;
}

void DracoStateEstimator::_FootContactUpdate(DracoSensorData* data) {
    if(data->rfoot_contact) sp_->b_rfoot_contact = 1;
    else sp_->b_rfoot_contact = 0;
    if(data->lfoot_contact) sp_->b_lfoot_contact = 1;
    else sp_->b_lfoot_contact = 0;
}
