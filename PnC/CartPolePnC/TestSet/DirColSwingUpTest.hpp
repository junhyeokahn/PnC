#pragma once

#include "Test.hpp"

#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

class RobotSystem;

class DirColSwingUpTest: public Test
{
private:
    std::unique_ptr< RigidBodyTree<double> > mTree;
    std::unique_ptr< drake::systems::RigidBodyPlant<double> > mPlant;

public:
    DirColSwingUpTest(RobotSystem*);
    virtual ~DirColSwingUpTest();

    virtual Eigen::VectorXd getTorqueInput();
    virtual void initialize();
};
