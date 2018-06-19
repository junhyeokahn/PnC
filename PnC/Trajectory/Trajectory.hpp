#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Eigen/Dense>
#include <assert.h>
#include <stdio.h>
#include <iostream>

class TrajectoryParam
{
public:
    TrajectoryParam(int dim_, double startTime_, double endTime_) {
        dim = dim_;
        startTime = startTime_;
        endTime = endTime_;
    };

    virtual ~TrajectoryParam() {};
    int dim;
    double startTime;
    double endTime;
};

class Trajectory
{
protected:
    TrajectoryParam* mTrajectoryParam;
    bool mIsTrajectoryGenerated;

    virtual void _generateTrajectory() = 0;
    virtual void _getPVA(double time_, Eigen::VectorXd & pos_,
                                       Eigen::VectorXd & vel_,
                                       Eigen::VectorXd & acc_) = 0;

public:
    Trajectory(TrajectoryParam* trajParam_) {
        mTrajectoryParam = trajParam_;
        mIsTrajectoryGenerated = false;
    }
    virtual ~Trajectory() {}

    virtual std::string getTrajectoryType()=0;
    void evalTrajectory(double time_, Eigen::VectorXd & pos_,
                                      Eigen::VectorXd & vel_,
                                      Eigen::VectorXd & acc_) {
        if (!mIsTrajectoryGenerated)
            _generateTrajectory();
        _getPVA(time_, pos_, vel_, acc_);
    }

    void deleteTrajectoryParam() {
        delete mTrajectoryParam;
    }
};

#endif /* TRAJECTORY_H */
