#include "QuaternionTrajectory.hpp"
#include <dart/dart.hpp>
#include "Utilities.hpp"

QuaternionTrajectory::QuaternionTrajectory(QuaternionTrajectoryParam* quatParam_):
    Trajectory((TrajectoryParam*)quatParam_) {
}

QuaternionTrajectory::~QuaternionTrajectory() {}

void QuaternionTrajectory::_generateTrajectory() {
    // Do Nothing
}

void QuaternionTrajectory::_getPVA(double time_, Eigen::VectorXd & pos_,
                                                 Eigen::VectorXd & vel_,
                                                 Eigen::VectorXd & acc_) {
    QuaternionTrajectoryParam* param = (QuaternionTrajectoryParam*) mTrajectoryParam;
    assert(pos_.size() = param->dim+1);
    assert(vel_.size() = param->dim);
    assert(acc_.size() = param->dim);
    assert(param->endTime > time_);

    if (time_ > param->startTime + param->dur) {
        pos_[0] = param->finQuaternion.w();
        pos_[1] = param->finQuaternion.x();
        pos_[2] = param->finQuaternion.y();
        pos_[3] = param->finQuaternion.z();
        vel_.setZero();
        acc_.setZero();
    } else {
        double t(myUtils::smooth_changing(0, 1, param->dur, time_));
        double t_dot(myUtils::smooth_changing_vel(0, 1, param->dur, time_));
        double t_ddot(myUtils::smooth_changing_acc(0, 1, param->dur, time_));
        Eigen::Quaternion<double> deltaQuat = (param->finQuaternion) * (param->initQuaternion.inverse());
        Eigen::Vector3d deltaSO3 = dart::math::quatToExp(deltaQuat);
        for (int i = 0; i < 3; ++i)
            deltaSO3[i] = myUtils::bind_half_pi(deltaSO3[i]);
        Eigen::Quaternion<double> quatDes = (dart::math::expToQuat(deltaSO3 * t)) * param->initQuaternion;
        Eigen::Vector3d angVelDes = deltaSO3 * t_dot;
        Eigen::Vector3d angAccDes = deltaSO3 * t_ddot;

        pos_[0] = quatDes.w();
        pos_[1] = quatDes.x();
        pos_[2] = quatDes.y();
        pos_[3] = quatDes.z();
        vel_ = angVelDes;
        acc_ = angAccDes;
    }
}

std::string QuaternionTrajectory::getTrajectoryType() {
    return "Quaternion";
}
