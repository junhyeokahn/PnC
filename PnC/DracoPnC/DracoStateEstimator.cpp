#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/DracoStateEstimator.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <Utils/Utilities.hpp>
#include <PnC/DracoPnC/StateEstimator/BasicAccumulation.hpp>
#include <PnC/DracoPnC/StateEstimator/BodyEstimator.hpp>
#include <Filter/filters.hpp>
#include <RobotSystem/RobotSystem.hpp>

DracoStateEstimator::DracoStateEstimator(RobotSystem* robot) {
    robot_ = robot;
    sp_ = DracoStateProvider::getStateProvider(robot_);
    curr_config_ = Eigen::VectorXd::Zero(robot_->getNumDofs());
    curr_qdot_= Eigen::VectorXd::Zero(robot_->getNumDofs());
    ori_est_ = new BasicAccumulation();

    //mocap_x_vel_est_ = new AverageFilter(SERVO_RATE, 0.01, 1.0);
    //mocap_y_vel_est_ = new AverageFilter(SERVO_RATE, 0.01, 1.5);
    //body_est_ = new BodyEstimator(robot);
}

DracoStateEstimator::~DracoStateEstimator(){
    delete ori_est_;
    //delete body_est_;
    //delete mocap_x_vel_est_;
    //delete mocap_y_vel_est_;
}

void DracoStateEstimator::initialization(DracoSensorData* data){
    curr_config_.setZero();
    curr_qdot_.setZero();

    // Joint Set
    for (int i(0); i<robot_->getNumActuatedDofs(); ++i){
        curr_config_[robot_->getNumVirtualDofs() + i] = data->q[i];
        curr_qdot_[robot_->getNumVirtualDofs() + i] = data->qdot[i];
    }
    std::vector<double> imu_acc(3);
    std::vector<double> imu_ang_vel(3);

    for(int i(0); i<3; ++i){
        imu_acc[i]  = data->imu_acc[i];
        imu_ang_vel[i] = data->imu_ang_vel[i];
    }

    Eigen::Vector3d body_ori;
    Eigen::Vector3d body_ang_vel;

    ori_est_->estimatorInitialization(imu_acc, imu_ang_vel);
    ori_est_->getEstimatedState(body_ori, body_ang_vel);
    //body_est_->Initialization(body_ori_);

    curr_config_[3] = body_ori[0];
    curr_config_[4] = body_ori[1];
    curr_config_[5] = body_ori[2];

    for(int i(0); i<3; ++i)  curr_qdot_[i+3] = body_ang_vel[i];

    robot_->updateSystem(curr_config_, curr_qdot_, false);

    //Eigen::VectorXd foot_pos =
        //robot_->getBodyNodeCoMIsometry(sp_->stance_foot).translation();
    //Eigen::VectorXd foot_vel =
        //robot_->getBodyNodeCoMSpatialVelocity(sp_->stance_foot).tail(3);
    Eigen::VectorXd foot_pos =
        robot_->getBodyNodeCoMIsometry(sp_->stance_foot).translation();
    Eigen::VectorXd foot_vel =
        robot_->getBodyNodeCoMSpatialVelocity(sp_->stance_foot).tail(3);

    curr_config_[0] = -foot_pos[0];
    curr_config_[1] = -foot_pos[1];
    curr_config_[2] = -foot_pos[2];
    curr_qdot_[0] = -foot_vel[0];
    curr_qdot_[1] = -foot_vel[1];
    curr_qdot_[2] = -foot_vel[2];

    robot_->updateSystem(curr_config_, curr_qdot_, true);

    sp_->q = curr_config_;
    sp_->qdot = curr_qdot_;

    // Right Contact
    if(data->rfoot_contact) sp_->b_rfoot_contact = 1;
    else sp_->b_rfoot_contact = 0;
    // Left Contact
    if(data->lfoot_contact) sp_->b_lfoot_contact = 1;
    else sp_->b_lfoot_contact = 0;

    sp_->saveCurrentData();
}

void DracoStateEstimator::update(DracoSensorData* data){
    curr_config_.setZero();
    curr_qdot_.setZero();

    for (int i(0); i<robot_->getNumActuatedDofs(); ++i){
        curr_config_[robot_->getNumVirtualDofs() + i] = data->q[i];
        curr_qdot_[robot_->getNumVirtualDofs() + i] = data->qdot[i];
    }
    std::vector<double> imu_acc(3);
    std::vector<double> imu_ang_vel(3);

    for(int i(0); i<3; ++i){
        imu_acc[i]  = data->imu_acc[i];
        imu_ang_vel[i] = data->imu_ang_vel[i];
    }

    Eigen::Vector3d body_ori;
    Eigen::Vector3d body_ang_vel;

    ori_est_->setSensorData( imu_acc, imu_ang_vel);
    ori_est_->getEstimatedState(body_ori, body_ang_vel);

    curr_config_[3] = body_ori[0];
    curr_config_[4] = body_ori[1];
    curr_config_[5] = body_ori[2];

    for(int i(0); i<3; ++i)  curr_qdot_[i+3] = body_ang_vel[i];

    robot_->updateSystem(curr_config_, curr_qdot_, false);

    //Eigen::VectorXd foot_pos =
        //robot_->getBodyNodeCoMIsometry(sp_->stance_foot).translation();
    //Eigen::VectorXd foot_vel =
        //robot_->getBodyNodeCoMSpatialVelocity(sp_->stance_foot).tail(3);
    Eigen::VectorXd foot_pos =
        robot_->getBodyNodeCoMIsometry(sp_->stance_foot).translation();
    Eigen::VectorXd foot_vel =
        robot_->getBodyNodeCoMSpatialVelocity(sp_->stance_foot).tail(3);

    curr_config_[0] = -foot_pos[0];
    curr_config_[1] = -foot_pos[1];
    curr_config_[2] = -foot_pos[2];
    curr_qdot_[0] = -foot_vel[0];
    curr_qdot_[1] = -foot_vel[1];
    curr_qdot_[2] = -foot_vel[2];

    robot_->updateSystem(curr_config_, curr_qdot_, true);

    sp_->q = curr_config_;
    sp_->qdot = curr_qdot_;

    //dynacore::pretty_print(sp_->q, std::cout, "state estimator config");

    // Right Contact
    if(data->rfoot_contact) sp_->b_rfoot_contact = 1;
    else sp_->b_rfoot_contact = 0;
    // Left Contact
    if(data->lfoot_contact) sp_->b_lfoot_contact = 1;
    else sp_->b_lfoot_contact = 0;

    sp_->saveCurrentData();

    // Mocap based body velocity
    Eigen::Vector3d mocap_body_vel;
    //body_foot_est_->update();
    //body_foot_est_->getMoCapBodyVel(mocap_body_vel);
}

