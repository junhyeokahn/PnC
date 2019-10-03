#include <dart/dart.hpp>

#include <Configuration.h>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/StateEstimator/IMUFrameEstimator.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

IMUFrameEstimator::IMUFrameEstimator(RobotSystem* robot) : filtered_acc_(3) {
    myUtils::pretty_constructor(2, "IMU Frame Estimator");
    robot_ = robot;

    worldTroot_ = Eigen::Isometry3d::Identity();
    world_root_rot_vel_ = Eigen::VectorXd::Zero(3);
    world_root_pris_vel_ = Eigen::VectorXd::Zero(3);

    imuTroot_ = Eigen::Isometry3d::Identity();
    imuQroot_ = Eigen::Quaternion<double>::Identity();
    worldQimu_ = Eigen::Quaternion<double>::Identity();
    imuQworld_prev_ = Eigen::Quaternion<double>::Identity();
    imuQworld_ = Eigen::Quaternion<double>::Identity();

    mag_alpha_ = 0.01;

    grav_accel_magnitude_ = 9.81;
    world_non_grav_accel_ = Eigen::VectorXd::Zero(3);
    non_gravitic_accel_frac_ = 0.;
    adaptive_threshold_min_ = 0.015;
    adaptive_threshold_max_ = 0.025;
    linear_stationary_factor_ = 0.;
    filter_alpha_ = 0.001;

    // TODO: find good cutoff frequency
    // TODO: do we need filters for angvel & mag?
    cutoff_freq_ = 2.0 * M_PI * 1.0;  // 1Hz // (2*pi*frequency) rads/s
    for (int i(0); i < 3; ++i) {
        filtered_acc_[i] =
            new digital_lp_filter(cutoff_freq_, DracoAux::ServoRate);
    }
}

void IMUFrameEstimator::estimatorInitialization(const Eigen::VectorXd& acc,
                                                const Eigen::VectorXd& ang_vel,
                                                const Eigen::VectorXd& mag) {
    for (int i(0); i < 3; ++i) {
        filtered_acc_[i]->input(acc[i]);
    }
    _InitIMUOrientationEstimateFromGravity();
    // TODO : compare
    //_InitIMUOrientationEstimateFromGravity2();

    // generate transform matrices
    Eigen::Isometry3d worldTimu = Eigen::Isometry3d(worldQimu_);
    imuTroot_ = robot_->getBodyNodeIsometry(DracoBodyNode::IMU);
    imuQroot_ = Eigen::Quaternion<double>(imuTroot_.linear());
    worldTroot_ = worldTimu * imuTroot_;

    // imu rot vel
    world_root_rot_vel_ = worldQimu_ * ang_vel;  // imu__est_w.vec();

    Eigen::Vector3d world__accel_measured = worldQimu_ * acc;
    Eigen::Vector3d world__accel_gravity =
        Eigen::Vector3d(0, 0, 1) * grav_accel_magnitude_;
    world_non_grav_accel_ = (world__accel_measured - world__accel_gravity);
    world_root_pris_vel_ =
        world_root_pris_vel_ + DracoAux::ServoRate * world_non_grav_accel_;
    worldTroot_.translation() = Eigen::Vector3d(0, 0, 0);
}

void IMUFrameEstimator::setSensorData(const Eigen::VectorXd& _acc,
                                      const Eigen::VectorXd& _ang_vel,
                                      const Eigen::VectorXd& _mag) {
    Eigen::VectorXd acc = Eigen::VectorXd::Zero(3);
    for (int i = 0; i < 3; ++i) {
        filtered_acc_[i]->input(_acc[i]);
        acc[i] = filtered_acc_[i]->output();
    }
    Eigen::VectorXd mag = _mag;

    // adaptation
    non_gravitic_accel_frac_ =
        std::abs(acc.norm() - grav_accel_magnitude_) / grav_accel_magnitude_;
    double threshold_shift =
        (adaptive_threshold_max_ + adaptive_threshold_min_) / 2.0;
    double full_accel_threshold = 0.95;
    double adaptive_scale = tan(-(full_accel_threshold - 0.5) * M_PI) /
                            (adaptive_threshold_min_ - threshold_shift);
    linear_stationary_factor_ =
        -atan((non_gravitic_accel_frac_ - threshold_shift) * adaptive_scale) /
            (M_PI) +
        0.5;  // 1.0 if no substantial linear acceleration, 0 if large linear
              // acceleration
    adaptive_alpha_ = filter_alpha_ * linear_stationary_factor_;

    // gyro
    Eigen::Vector3d ang_vel = _ang_vel;

    Eigen::Quaterniond imu__w =
        Eigen::Quaternion<double>(0, ang_vel[0], ang_vel[1], ang_vel[2]);
    Eigen::Quaterniond imuQworld__w_dot = imu__w * imuQworld_;
    Eigen::Quaterniond imuQworld__w;
    imuQworld__w.w() =
        imuQworld_.w() + -0.5 * imuQworld__w_dot.w() * DracoAux::ServoRate;
    imuQworld__w.x() =
        imuQworld_.x() + -0.5 * imuQworld__w_dot.x() * DracoAux::ServoRate;
    imuQworld__w.y() =
        imuQworld_.y() + -0.5 * imuQworld__w_dot.y() * DracoAux::ServoRate;
    imuQworld__w.z() =
        imuQworld_.z() + -0.5 * imuQworld__w_dot.z() * DracoAux::ServoRate;

    // accel
    Eigen::Vector3d world__acc_norm_pred = worldQimu_ * acc.normalized();
    Eigen::Quaterniond world__w_acc = Eigen::Quaternion<double>(
        sqrt((world__acc_norm_pred[2] + 1.0) / 2.0),
        -world__acc_norm_pred[1] / sqrt(2 * (world__acc_norm_pred[2] + 1)),
        world__acc_norm_pred[0] / sqrt(2 * (world__acc_norm_pred[2] + 1)), 0);

    Eigen::Quaterniond world__w_acc_filt = world__w_acc.slerp(
        1.0 - adaptive_alpha_, Eigen::Quaterniond::Identity());

    // mag
    Eigen::Vector3d world__mag_norm_pred = worldQimu_ * mag.normalized();
    double mag_gamma = world__mag_norm_pred.x() * world__mag_norm_pred.x() +
                       world__mag_norm_pred.y() * world__mag_norm_pred.y();
    Eigen::Quaterniond world__w_mag = Eigen::Quaternion<double>(
        sqrt(mag_gamma + world__mag_norm_pred.x() * sqrt(mag_gamma)) /
            sqrt(2 * mag_gamma),
        0, 0,
        world__mag_norm_pred.y() /
            sqrt(2 * (mag_gamma + world__mag_norm_pred.x() * sqrt(mag_gamma))));
    Eigen::Quaterniond world__w_mag_filt =
        world__w_mag.slerp(1.0 - mag_alpha_, Eigen::Quaterniond::Identity());

    imuQworld_ = imuQworld__w * world__w_acc_filt * world__w_mag_filt;
    imuQworld_.normalize();
    worldQimu_ = imuQworld_.inverse();

    // generate transform matrices
    Eigen::Isometry3d worldTimu = Eigen::Isometry3d(worldQimu_);
    imuTroot_ = robot_->getBodyNodeIsometry(DracoBodyNode::IMU);
    imuQroot_ = Eigen::Quaternion<double>(imuTroot_.linear());
    worldTroot_ = worldTimu * imuTroot_;

    // est imu rot vel
    // Eigen::Quaterniond world__est_w_dot;
    // world__est_w_dot.w() = (imuQworld_prev_.w() - imuQworld_.w()) * 2.0 /
    // DracoAux::ServoRate; world__est_w_dot.x() = (imuQworld_prev_.x() -
    // imuQworld_.x())
    // * 2.0 / DracoAux::ServoRate; world__est_w_dot.y() = (imuQworld_prev_.y()
    // - imuQworld_.y()) * 2.0 / DracoAux::ServoRate; world__est_w_dot.z() =
    // (imuQworld_prev_.z() - imuQworld_.z()) * 2.0 / DracoAux::ServoRate;
    // Eigen::Quaterniond imu__est_w = world__est_w_dot *
    // imuQworld_prev_.inverse(); m_imuQworld__prev = imuQworld_;
    world_root_rot_vel_ = worldQimu_ * ang_vel;  // imu__est_w.vec();

    // est imu lin vel
    Eigen::Vector3d world__accel_measured = worldQimu_ * acc;
    Eigen::Vector3d world__accel_gravity =
        Eigen::Vector3d(0, 0, 1) * grav_accel_magnitude_;
    world_non_grav_accel_ = (world__accel_measured - world__accel_gravity);
    world_root_pris_vel_ =
        world_root_pris_vel_ + DracoAux::ServoRate * world_non_grav_accel_;
    worldTroot_.translation() = Eigen::Vector3d(0, 0, 0);
    // worldTroot_.translation() = m_world__root_pris_pos +
    // m_world__root_pris_vel*DracoAux::ServoRate +
    // 0.5*world_non_grav_accel_*DracoAux::ServoRate*DracoAux::ServoRate;
    // TODO : Check position
}

void IMUFrameEstimator::_InitIMUOrientationEstimateFromGravity() {
    Eigen::Vector3d g_B;
    g_B.setZero();
    for (int i(0); i < 3; ++i) {
        // We expect a negative number if gravity is pointing opposite of
        // the IMU direction
        g_B[i] = filtered_acc_[i]->output();
    }
    // Test Vector  ////////////////////////
    // g_B[0] = 0.1; g_B[1] = 0.4; g_B[2] = 0.5;
    // g_B[0] = 0.0; g_B[1] = 0.0; g_B[2] = 1.0;
    // qd rot_quat;
    // dynacore::convert(1.0, -0.5, 0.1, rot_quat);
    // Eigen::Matrix3d rot_mt(rot_quat);
    // g_B = rot_mt * g_B;
    //////////////////////////////////////////////
    g_B.normalize();

    double theta_pitch = atan2(g_B[0], g_B[2]);

    Eigen::Vector3d g_B_xz = g_B;
    g_B_xz[1] = 0.;
    g_B_xz.normalize();

    double inner = g_B_xz.transpose() * g_B;
    double theta_roll = acos(inner);
    if (g_B[1] > 0) theta_roll *= (-1.);

    // Eigen::Quaternion<double> quat_pitch(dart::math::eulerYZXToMatrix(
    // Eigen::Vector3d(0., theta_pitch, 0.)));
    Eigen::Quaternion<double> quat_pitch(
        dart::math::eulerYZXToMatrix(Eigen::Vector3d(theta_pitch, 0., 0.)));
    Eigen::Quaternion<double> quat_roll(
        dart::math::eulerYZXToMatrix(Eigen::Vector3d(0., 0., theta_roll)));
    Eigen::Quaternion<double> local2Glob = quat_pitch * quat_roll;

    worldQimu_ = local2Glob.inverse();
    imuQworld_ = local2Glob;
    imuQworld_prev_ = imuQworld_;

    ///////////////////////////////////////////////////////////////////
    // Eigen::Matrix3d Global_ori(global_ori_quat_);
    // v3 check_vec = Global_ori * g_B;
    // dynacore::pretty_print(check_vec, std::cout, "check vec");
    // dynacore::pretty_print(g_B, std::cout, "g_B");
    // printf("pitch, roll: %f, %f\n", theta_pitch, theta_roll);
}

void IMUFrameEstimator::_InitIMUOrientationEstimateFromGravity2() {
    Eigen::VectorXd acc = Eigen::VectorXd::Zero(3);
    for (int i = 0; i < 3; ++i) {
        acc[i] = filtered_acc_[i]->output();
    }
    worldQimu_ = Eigen::Quaterniond::FromTwoVectors(acc.normalized(),
                                                    Eigen::Vector3d(0, 0, 1));
    worldQimu_ = Eigen::AngleAxisd(-myUtils::QuatToYaw(worldQimu_),
                                   Eigen::Vector3d::UnitZ()) *
                 worldQimu_;  // initialize yaw to face forwards
    imuQworld_ = worldQimu_.inverse();
    imuQworld_prev_ = imuQworld_;
}

Eigen::VectorXd IMUFrameEstimator::_so3_to_euler_zyx_dot(
    const Eigen::VectorXd& _global_ori_ypr,
    const Eigen::VectorXd& _global_ang_vel) {
    Eigen::MatrixXd so3_to_euler_zyx_dot_map(3, 3);
    double x(_global_ori_ypr[2]);
    double y(_global_ori_ypr[1]);
    double z(_global_ori_ypr[0]);

    if (y == M_PI / 2.0) {
        std::cout << "Singular at mapping from euler zyx to so3" << std::endl;
        exit(0);
    }

    so3_to_euler_zyx_dot_map << cos(z) * sin(y) / cos(y),
        sin(y) * sin(z) / cos(y), 1, -sin(z), cos(z), 0, cos(z) / cos(y),
        sin(z) / cos(y), 0;
    return so3_to_euler_zyx_dot_map * _global_ang_vel;
}

void IMUFrameEstimator::getVirtualJointPosAndVel(Eigen::VectorXd& vq,
                                                 Eigen::VectorXd& vqdot) {
    vq = Eigen::VectorXd::Zero(6);
    vqdot = Eigen::VectorXd::Zero(6);

    Eigen::VectorXd euler_zyx =
        dart::math::matrixToEulerZYX(imuTroot_.linear());
    // euler zyx
    for (int i = 0; i < 3; ++i) {
        vq[i + 3] = euler_zyx[i];
    }
    Eigen::VectorXd euler_zyx_dot =
        _so3_to_euler_zyx_dot(euler_zyx, world_root_rot_vel_);
    // euler zyx dot
    for (int i = 0; i < 3; ++i) {
        vqdot[i] = world_root_pris_vel_[i];
        vqdot[i + 3] = euler_zyx_dot[i];
    }
}
