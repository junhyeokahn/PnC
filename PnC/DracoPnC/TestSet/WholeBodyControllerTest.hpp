#pragma once

#include "Test.hpp"
#include "DracoPnC/DracoInterface.hpp"
#include "BSplineBasic.h"

class WBLC;
class WBLC_ExtraData;
class WBLCContact;
class Task;

class WholeBodyControllerTest: public Test
{
public:
    WholeBodyControllerTest (RobotSystem* robot_);
    virtual ~WholeBodyControllerTest ();

    virtual void getTorqueInput(void* commandData_);
    virtual void initialize();

private:
    WBLC* mWBLC;
    WBLC_ExtraData* mWBLCExtraData;
    Task* mCoMTask;
    WBLCContact* mRfContact;
    WBLCContact* mLfContact;
    std::vector<Task*> mTaskList;
    std::vector<WBLCContact*> mContactList;

    BS_Basic<3, 3, 0, 2, 2> mSpline;
    Eigen::Vector3d mMid;
    Eigen::Vector3d mAmp;
    Eigen::Vector3d mFreq;
    double mInterpolationDuration;
    double mTestInitTime;

    void _updateContact(); // update mContactList
    void _updateTask(); // update mTaskList
    void _WBLCpreProcess(); // Set dynamic properties and cost
    void _WBLCpostProcess(); // unset task and contact
};
