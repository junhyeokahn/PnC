#pragma once

#include "Test.hpp"


class RobotSystem;
class Planner;

class DirColSwingUpTest: public Test
{
private:
    Planner* mPlanner;

public:
    DirColSwingUpTest(RobotSystem*);
    virtual ~DirColSwingUpTest();

    virtual Eigen::VectorXd getTorqueInput();
};
