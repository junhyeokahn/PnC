#pragma once

#include "Test.hpp"
#include "DracoHipPnC/DracoHipInterface.hpp"

class GravityCompensationTest: public Test
{
public:
    GravityCompensationTest (RobotSystem* robot_);
    virtual ~GravityCompensationTest ();

    virtual void getTorqueInput(void* commandData_);
    virtual void initialize();

private:

};
