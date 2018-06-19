#ifndef QUATERNIONTRAJECTORY_H
#define QUATERNIONTRAJECTORY_H

#include "Trajectory.hpp"

class QuaternionTrajectoryParam: public TrajectoryParam
{
public:
    QuaternionTrajectoryParam(int dim_, double startTime_, double endTime_):
        TrajectoryParam(dim_, startTime_, endTime_) {
    };
    virtual ~QuaternionTrajectoryParam() {};

    Eigen::Quaternion<double> initQuaternion;
    Eigen::Quaternion<double> finQuaternion;
    double dur;
};

class QuaternionTrajectory: public Trajectory
{
protected:
    virtual void _generateTrajectory();
    virtual void _getPVA(double time_, Eigen::VectorXd & pos_,
                                       Eigen::VectorXd & vel_,
                                       Eigen::VectorXd & acc_);

public:
    QuaternionTrajectory(QuaternionTrajectoryParam* quatParam_);
    virtual ~QuaternionTrajectory();
    virtual std::string getTrajectoryType();


};

#endif /* QUATERNIONTRAJECTORY_H */
