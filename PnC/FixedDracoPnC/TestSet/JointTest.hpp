#pragma once

#include "Test.hpp"
#include "BSplineBasic.h"
#include "FixedDracoPnC/FixedDracoInterface.hpp"

class JointTest: public Test
{
public:
    JointTest (RobotSystem* robot_);
    virtual ~JointTest ();

    virtual void getTorqueInput(void* commandData_);
    virtual void initialize();

private:
    Eigen::VectorXd mTestInitQ;
    BS_Basic<10, 3, 0, 2, 2> mSpline;
    Eigen::VectorXd mMid;
    Eigen::VectorXd mAmp;
    Eigen::VectorXd mFreq;
    Eigen::VectorXd mKp;
    Eigen::VectorXd mKd;
    double mInterpolationDuration;
    double mTestInitTime;

    // Debugging Data
    Eigen::VectorXd q_des_debug;
    Eigen::VectorXd qdot_des_debug;
    Eigen::VectorXd qddot_des_debug;
    Eigen::VectorXd q_act_debug;
    Eigen::VectorXd qdot_act_debug;
};
