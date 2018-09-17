#pragma once

#include "PnC/Test.hpp"
#include "Utils/BSplineBasic.h"
#include "PnC/FixedDracoPnC/FixedDracoInterface.hpp"

class OSCTest: public Test
{
public:
    OSCTest (RobotSystem* robot_);
    virtual ~OSCTest ();

    virtual void getTorqueInput(void* commandData_);
    virtual void initialize();

private:
    Eigen::VectorXd mTestInitQ;
    Eigen::Vector3d mTestInitRFPos;
    BS_Basic<3, 3, 0, 2, 2> mSpline;
    Eigen::Vector3d mMid;
    Eigen::Vector3d mAmp;
    Eigen::Vector3d mFreq;
    Eigen::VectorXd mKpRf;
    Eigen::VectorXd mKdRf;
    Eigen::VectorXd mKpQ;
    Eigen::VectorXd mKdQ;
    double mInterpolationDuration;
    double mTestInitTime;

    // Debugging Data
    Eigen::VectorXd rf_pos_des_debug;
    Eigen::VectorXd rf_vel_des_debug;
    Eigen::VectorXd rf_acc_des_debug;
    Eigen::VectorXd rf_pos_act_debug;
    Eigen::VectorXd rf_vel_act_debug;
};
