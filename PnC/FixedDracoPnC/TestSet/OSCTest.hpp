#pragma once

#include "Test.hpp"
#include "BSplineBasic.h"
#include "FixedDracoPnC/FixedDracoInterface.hpp"

class OSCTest: public Test
{
public:
    OSCTest (RobotSystem* robot_);
    virtual ~OSCTest ();

    virtual void getTorqueInput(void* commandData_);
    virtual void initialize();

private:
    Eigen::VectorXd mInitQ;
    BS_Basic<3, 3, 0, 2, 2> mSpline;
    Eigen::Vector3d mMid;
    Eigen::Vector3d mAmp;
    Eigen::Vector3d mFreq;
    double mInterpolationDuration;
    double mTestInitTime;

};
