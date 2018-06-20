#pragma once

#include "Test.hpp"

class RobotSystem;

class GravityCompensationTest: public Test
{
private:

public:
    GravityCompensationTest(RobotSystem*);
    virtual ~GravityCompensationTest();

    virtual Eigen::VectorXd getTorqueInput();
};
