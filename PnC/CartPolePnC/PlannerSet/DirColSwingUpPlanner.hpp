#pragma once
#include "Planner.hpp"

#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

class DirColSwingUpPlanningParameter : public PlanningParameter
{
public:
    DirColSwingUpPlanningParameter (double startTime_, double endTime_) :
        PlanningParameter(startTime_, endTime_) {};
    virtual ~DirColSwingUpPlanningParameter ();
};

class DirColSwingUpPlanner : public Planner
{
public:
    DirColSwingUpPlanner (
            std::shared_ptr<DirColSwingUpPlanningParameter> param);
    virtual ~DirColSwingUpPlanner ();

private:
    std::unique_ptr< RigidBodyTree<double> > mTree;
    std::unique_ptr< drake::systems::RigidBodyPlant<double> > mPlant;

    virtual void _doPlan();
    virtual void _evalTrajecotry( double time, Eigen::VectorXd & pos,
                                  Eigen::VectorXd & vel, Eigen::VectorXd acc,
                                  Eigen::VectorXd &eff );
};
