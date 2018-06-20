#pragma once

#include <Eigen/Dense>
#include <stdio.h>
#include <iostream>
#include <vector>

class RobotSystem;

class Test
{
protected:
    RobotSystem* mRobot;

    bool mIsInitialized;

public:
    Test(RobotSystem* robot_): mIsInitialized(false) {
        mRobot = robot_;
    };
    virtual ~Test() {};

    virtual Eigen::VectorXd getTorqueInput() = 0;
};
