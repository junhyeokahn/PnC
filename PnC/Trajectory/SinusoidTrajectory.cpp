#include "SinusoidTrajectory.hpp"

SinusoidTrajectory::SinusoidTrajectory(SinusoidTrajectoryParam* sinuParam_):
    Trajectory((TrajectoryParam*)sinuParam_) {
}

SinusoidTrajectory::~SinusoidTrajectory() {}

void SinusoidTrajectory::_generateTrajectory() {
    // Do Nothing
}

void SinusoidTrajectory::_getPVA(double time_, Eigen::VectorXd & pos_,
                                               Eigen::VectorXd & vel_,
                                               Eigen::VectorXd & acc_) {
    SinusoidTrajectoryParam* param = (SinusoidTrajectoryParam*) mTrajectoryParam;
    assert(pos_.size() = param->dim);
    assert(vel_.size() = param->dim);
    assert(acc_.size() = param->dim);
    assert(param->endTime > time_);
    for (int i = 0; i < param->dim; ++i) {
        double omega(2*M_PI*param->frequency[i]);
        pos_[i] = param->center[i] + param->amp[i] * sin(omega*(time_-param->startTime));
        vel_[i] = omega * param->amp[i] * cos(omega*(time_-param->startTime));
        acc_[i] = -1 * omega * omega * param->amp[i] * sin(omega*(time_-param->startTime));
    }
}

std::string SinusoidTrajectory::getTrajectoryType() {
    return "Sinusoid";
}
