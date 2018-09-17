#pragma once

#include "PnC/Test.hpp"

class WBLC;
class WBLC_ExtraData;
class WBLCContact;
class Task;

class CentroidKinematicOptimizationTest: public Test
{
public:
    CentroidKinematicOptimizationTest (RobotSystem* robot_);
    virtual ~CentroidKinematicOptimizationTest ();

    virtual void getTorqueInput(void* commandData_);
    virtual void initialize();

private:
    double mTestInitTime;
    Eigen::VectorXd mTestInitQ;
    BS_Basic<10, 3, 0, 2, 2> mSpline;
    double mInterpolationDuration;
    Eigen::VectorXd mInterpolationPosition;

    // Planner
    //Planner* planner;

    // Controller
    WBLC* mWBLC;
    WBLC_ExtraData* mWBLCExtraData;
    Task* mCoMTask;
    Task* mJointTask;
    WBLCContact* mRfContact;
    WBLCContact* mLfContact;
    std::vector<Task*> mTaskList;
    std::vector<WBLCContact*> mContactList;
    void _updateContact(); // update mContactList
    void _updateTask(); // update mTaskList
    void _WBLCpreProcess(); // Set dynamic properties and cost
    void _WBLCpostProcess(); // unset task and contact


};
