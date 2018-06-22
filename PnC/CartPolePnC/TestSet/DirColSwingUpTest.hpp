#pragma once

#include "Test.hpp"

class RobotSystem;
class Planner;
class PlanningParameter;

class DirColSwingUpTest: public Test
{
private:
    Planner* mPlanner;

public:
    DirColSwingUpTest(RobotSystem*);
    virtual ~DirColSwingUpTest();

    virtual Eigen::VectorXd getTorqueInput();
    virtual void initialize();

    Eigen::VectorXd mPosDes;
    Eigen::VectorXd mVelDes;
    double mEffDes;
    Eigen::VectorXd mPosAct;
    Eigen::VectorXd mVelAct;
    Eigen::VectorXd mEffAct;
};
