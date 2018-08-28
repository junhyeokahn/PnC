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

class InvKinTest: public Test
{
public:
    InvKinTest (RobotSystem* robot_);
    virtual ~InvKinTest ();

    virtual void getTorqueInput(void* commandData_);
    virtual void initialize();

private:
    std::unique_ptr<RigidBodyTree<double>> mDrakeModel;
    Eigen::VectorXd mInitQ;
    int mRfIdx;
    int mLfIdx;
    int mWorldIdx;
    Eigen::Isometry3d mInitRfIso;
    Eigen::Isometry3d mInitLfIso;
    Eigen::Vector3d mInitCOM;
};
