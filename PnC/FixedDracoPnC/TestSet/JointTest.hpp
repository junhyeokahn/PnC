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
    Eigen::VectorXd mInitQ;
    BS_Basic<10, 10, 0, 2, 2> mSpline;
    Eigen::VectorXd mMid;
    Eigen::VectorXd mAmp;
    Eigen::VectorXd mFreq;
    double mInterpolationDuration;
    double mTestInitTime;
};
