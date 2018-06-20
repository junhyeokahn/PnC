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


public:
    Test(RobotSystem* robot_): isInitialized(false) {
        mRobot = robot_;
    };
    virtual ~Test() {
    };
    virtual Eigen::VectorXd getTorqueInput() = 0;
    virtual void initialize() = 0;

    bool isInitialized;
};
