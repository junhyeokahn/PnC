#pragma once

#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/Filters/Basic/filters.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>

class IMUFrameEstimator {
   public:
    IMUFrameEstimator(RobotSystem* robot);
    ~IMUFrameEstimator() {}

    void estimatorInitialization(const Eigen::VectorXd& acc,
                                 const Eigen::VectorXd& ang_vel,
                                 const Eigen::VectorXd& mag);

    void setSensorData(const Eigen::VectorXd& acc,
                       const Eigen::VectorXd& ang_vel,
                       const Eigen::VectorXd& mag);

    void getVirtualJointPosAndVel(Eigen::VectorXd& vq, Eigen::VectorXd& vqdot);

   protected:
    RobotSystem* robot_;

    double cutoff_freq_;
    std::vector<filter*> filtered_acc_;

    void _InitIMUOrientationEstimateFromGravity();
    void _InitIMUOrientationEstimateFromGravity2();

    Eigen::VectorXd _so3_to_euler_zyx_dot(
        const Eigen::VectorXd& _global_ori_ypr,
        const Eigen::VectorXd& _global_ang_vel);

    // estimate
    Eigen::Isometry3d worldTroot_;
    Eigen::VectorXd world_root_rot_vel_;
    Eigen::VectorXd world_root_pris_vel_;

    // intermediate values
    Eigen::Isometry3d imuTroot_;
    Eigen::Quaterniond imuQroot_;
    Eigen::Quaterniond worldQimu_;
    Eigen::Quaterniond imuQworld_prev_;
    Eigen::Quaterniond imuQworld_;

    // mag
    double mag_alpha_;

    // accel
    double grav_accel_magnitude_;
    Eigen::VectorXd world_non_grav_accel_;
    double non_gravitic_accel_frac_;
    double adaptive_threshold_min_;
    double adaptive_threshold_max_;
    double linear_stationary_factor_;
    double filter_alpha_;
    double adaptive_alpha_;
};
