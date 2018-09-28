#pragma once

#include "PnC/Test.hpp"

class WBLC;
class WBLC_ExtraData;
class WBLCContact;
class Task;

class AdmittanceTest : public Test
{
public:
    AdmittanceTest (RobotSystem* robot_);
    virtual ~AdmittanceTest ();

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
    WBLCContact* mRfContact;
    WBLCContact* mLfContact;
    std::vector<Task*> mTaskList;
    std::vector<WBLCContact*> mContactList;
    void _updateContact(); // update mContactList
    void _updateTask(); // update mTaskList
    void _WBLCpreProcess(); // Set dynamic properties and cost
    void _WBLCpostProcess(); // unset task and contact

    Eigen::VectorXd mCentroidTaskKp;
    Eigen::VectorXd mCentroidTaskKd;
    Eigen::VectorXd mJointTaskKp;
    Eigen::VectorXd mJointTaskKd;
    Eigen::VectorXd mNominalCentroidState;
    Eigen::VectorXd mNominalJointState;
    double mInterpolationDuration;
    Eigen::VectorXd mAmp;
    Eigen::VectorXd mMid;
    Eigen::VectorXd mFreq;

};
