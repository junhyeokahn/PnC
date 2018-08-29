#pragma once

#include "Test.hpp"

#include <drake/common/find_resource.h>
#include <drake/multibody/ik_options.h>
#include <drake/multibody/joints/floating_base_types.h>
#include <drake/multibody/parsers/urdf_parser.h>
#include <drake/multibody/rigid_body_constraint.h>
#include <drake/multibody/rigid_body_ik.h>
#include <drake/multibody/rigid_body_tree.h>
#include <memory>
#include "BSplineBasic.h"

class InvKinTest: public Test
{
public:
    InvKinTest (RobotSystem* robot_);
    virtual ~InvKinTest ();

    virtual void getTorqueInput(void* commandData_);
    virtual void initialize();

private:
    // inverse kinematics
    std::unique_ptr<RigidBodyTree<double>> mDrakeModel;
    Eigen::VectorXd mInitQ;
    Eigen::VectorXd mPrevSol;
    int mRfIdx;
    int mLfIdx;
    int mWorldIdx;
    Eigen::Isometry3d mInitRfIso;
    Eigen::Isometry3d mInitLfIso;
    Eigen::Vector3d mInitCOM;

    // whole body controller
    WBLC* mWBLC;
    WBLC_ExtraData* mWBLCExtraData;
    Task* mJointTask;
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
    void _WBLCpreProcess(); // Set dynamic properties and cost
    void _WBLCpostProcess(); // unset task and contact

};
