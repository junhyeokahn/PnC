#ifndef SINUSOIDTRAJECTORY_H
#define SINUSOIDTRAJECTORY_H

#include "Trajectory.hpp"

class SinusoidTrajectoryParam: public TrajectoryParam
{
public:
    SinusoidTrajectoryParam(int dim_, double startTime_, double endTime_):
        TrajectoryParam(dim_, startTime_, endTime_) {
        center = Eigen::VectorXd::Zero(dim_);
        amp = Eigen::VectorXd::Zero(dim_);
        frequency = Eigen::VectorXd::Zero(dim_);
    };
    virtual ~SinusoidTrajectoryParam() {};

    Eigen::VectorXd center;
    Eigen::VectorXd amp;
    Eigen::VectorXd frequency;
};

class SinusoidTrajectory: public Trajectory
{
protected:
    virtual void _generateTrajectory();
    virtual void _getPVA(double time_, Eigen::VectorXd & pos_,
                                       Eigen::VectorXd & vel_,
                                       Eigen::VectorXd & acc_);

public:
    SinusoidTrajectory(SinusoidTrajectoryParam* sinuParam_);
    virtual ~SinusoidTrajectory();
    virtual std::string getTrajectoryType();


};

#endif /* SINUSOIDTRAJECTORY_H */
