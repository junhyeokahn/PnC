#ifndef TEST_H
#define TEST_H

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
    virtual void initialize() = 0;
};

#endif /* TEST_H */
