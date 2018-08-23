#pragma once

#include "Test.hpp"

class CentroidKinematicOptimizationTest: public Test
{
public:
    CentroidKinematicOptimizationTest (RobotSystem* robot_);
    virtual ~CentroidKinematicOptimizationTest ();

    virtual void getTorqueInput(void* commandData_);
    virtual void initialize();

private:

};
