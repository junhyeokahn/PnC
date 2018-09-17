#pragma once

#include "PnC/Test.hpp"
#include "PnC/CartPolePnC/CartPoleInterface.hpp"

class RobotSystem;
class Planner;
class PlanningParameter;
class CartPoleCommand;

class DirColSwingUpTest: public Test
{
private:
    Planner* mPlanner;

public:
    DirColSwingUpTest(RobotSystem*);
    virtual ~DirColSwingUpTest();

    virtual void getTorqueInput(void* commandData_);
    virtual void initialize();
};
