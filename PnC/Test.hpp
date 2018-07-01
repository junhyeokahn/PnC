#pragma once

#include <Eigen/Dense>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <memory>

class RobotSystem;

class Test
{
protected:
    RobotSystem* mRobot;

public:
    Test(RobotSystem* robot_): isInitialized(false) {
        mRobot = robot_;
    };
    virtual ~Test() {};
    virtual void getTorqueInput(void* commandData_) = 0;
    virtual void initialize() = 0;

    bool isInitialized;
};
