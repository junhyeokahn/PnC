#include <PnC/DracoPnC/DracoMoCapManager.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/StateEstimator/BodyEstimator.hpp>
#include <Utils/IO/DataManager.hpp>

BodyEstimator::BodyEstimator(RobotSystem* robot) {
    myUtils::pretty_constructor(2, "Body Estimator");

    mocap_manager_ = new DracoMoCapManager(robot);
    mocap_manager_->start();

    robot_ = robot;

    body_led_vel_.setZero();
    for (int i(0); i < 3; ++i) {
        vel_filter_.push_back(
            new deriv_lp_filter(2. * 50. * M_PI, DracoAux::ServoRate));
    }

    DataManager::GetDataManager()->RegisterData(&body_led_vel_, VECT3,
                                                "Body_LED_vel", 3);
    sp_ = DracoStateProvider::getStateProvider(robot_);
}

BodyEstimator::~BodyEstimator() {
    delete mocap_manager_;
    for (int i = 0; i < 3; ++i) {
        delete vel_filter_[i];
    }
}

void BodyEstimator::getMoCapBodyPos(const Eigen::Quaternion<double>& body_ori,
                                    Eigen::Vector3d& local_body_pos) {
    // Body LED offset accunt
    Eigen::Matrix3d Body_rot(body_ori);
    Eigen::Vector3d body_led_offset;
    body_led_offset.setZero();
    body_led_offset[0] = -0.075;
    local_body_pos += Body_rot * body_led_offset;
}

void BodyEstimator::Update() {
    for (int i(0); i < 3; ++i) {
        vel_filter_[i]->input(mocap_manager_->led_pos_data_[i]);
        body_led_vel_[i] = vel_filter_[i]->output();
    }
}

void BodyEstimator::Initialization(const Eigen::Quaternion<double>& body_ori) {
    mocap_manager_->imu_body_ori_ = body_ori;
    mocap_manager_->CoordinateUpdateCall();
}

void BodyEstimator::getMoCapBodyOri(Eigen::Quaternion<double>& quat) {
    quat = mocap_manager_->body_quat_;
}

void BodyEstimator::getMoCapBodyVel(Eigen::Vector3d& body_vel) {
    body_vel = body_led_vel_;
}
