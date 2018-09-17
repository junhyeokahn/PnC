#pragma once

#include "PnC/Test.hpp"

class SymExpValidationTest: public Test
{
public:
    SymExpValidationTest (RobotSystem* robot_);
    virtual ~SymExpValidationTest ();

    virtual void getTorqueInput(void* commandData_);
    virtual void initialize();

private:

};
