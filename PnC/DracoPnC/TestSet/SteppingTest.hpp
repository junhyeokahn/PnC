#pragma once

#include "PnC/Test.hpp"

class WBLC;
class WBLC_ExtraData;
class WBLCContact;
class Task;

class SteppingTest : public Test
{
public:
    SteppingTest (RobotSystem* robot_);
    virtual ~SteppingTest ();

    virtual void getTorqueInput(void* commandData_);
    virtual void initialize();

private:
    Eigen::VectorXd mTestInitQ;
    Eigen::VectorXd mTestInitCoMPos;
    double mTestInitTime;

    WBLC* mWBLC;
    WBLC_ExtraData* mWBLCExtraData;

    Task* mCentroidTask;
    Task* mJointTask;
    Task* mCoMRPYTask;

    WBLCContact* mRfContact;
    WBLCContact* mLfContact;
    WBLCContact* mFixedBodyContact;

    std::vector<Task*> mTaskList;
    std::vector<WBLCContact*> mContactList;

    void _updateContact(); // update mContactList
    void _updateTask(); // update mTaskList
    void _WBLCpreProcess(); // Set dynamic properties and cost
    void _WBLCpostProcess(); // unset task and contact
    void _updateContactListandTaskList();
    void _updatePhase();

    Eigen::VectorXd mCentroidTaskKp;
    Eigen::VectorXd mCentroidTaskKd;
    Eigen::VectorXd mJointTaskKp;
    Eigen::VectorXd mJointTaskKd;
    Eigen::VectorXd mCoMRPYTaskKp;
    Eigen::VectorXd mCoMRPYTaskKd;

    // phase
    int mNumPhase;
    int mCurrentPhase;
    double mPhaseStartTime;
    double mPhaseEndTime;
    Eigen::VectorXd mPhaseTimeHorizon;
    double mWaitTimeBtwPhase;
    Eigen::VectorXd mInitJointTask;
    Eigen::VectorXd mInitCentroidTask;
    Eigen::VectorXd mTargetJointTask;
    Eigen::VectorXd mTargetCentroidTask;
    Eigen::VectorXd mTargetCoMRPYTask;
    bool mDoUpdateContactListAndTaskList;
};
