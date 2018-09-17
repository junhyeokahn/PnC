#pragma once
#include "PnC/Planner.hpp"

#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

class DirColSwingUpPlanningParameter : public PlannerParameter
{
public:
    DirColSwingUpPlanningParameter () : PlannerParameter() {};
    virtual ~DirColSwingUpPlanningParameter () {};

    Eigen::VectorXd initialState;
    Eigen::VectorXd finalState;
    int numTimeSample;
    double minimumTimeStep;
    double maximumTimeStep;
    double torqueLimit;
    double R;
    double timeSpanInit;
};


class DirColSwingUpPlanner : public Planner
{
public:
    DirColSwingUpPlanner ();
    virtual ~DirColSwingUpPlanner ();

    // file name looks like dircol_time_state_input
    void saveTrajectory(const std::string & planner_type) {
        double startTime = mParam->startTime;
        double endTime = mParam->endTime;
        int numEval = std::floor((endTime - startTime) / SERVO_RATE);
        Eigen::VectorXd pos = Eigen::VectorXd::Zero(mParam->numState / 2 );
        Eigen::VectorXd vel = Eigen::VectorXd::Zero(mParam->numState / 2);
        Eigen::VectorXd input = Eigen::VectorXd::Zero(mParam->numInput);
        double evalTime(0.0);
        Eigen::VectorXd aug =
            Eigen::VectorXd::Zero(1 + mParam->numState + mParam->numInput);
        std::string file_name = planner_type;
        file_name += "_";
        file_name += "1";
        file_name += "_";
        file_name += std::to_string(mParam->numState);
        file_name += "_";
        file_name += std::to_string(mParam->numInput);

        evalTime = startTime;
        for (int i = 0; i < numEval; ++i) {
            getPlan(evalTime, pos, vel, input);
            aug = Eigen::VectorXd::Zero(1 + mParam->numState + mParam->numInput);
            aug << evalTime, pos, vel, input;
            myUtils::saveVector(aug, file_name, true);
            evalTime += SERVO_RATE;
        }
        std::cout << "[Trajectory Saved]" << std::endl;
    }

private:
    std::unique_ptr< RigidBodyTree<double> > mTree;
    std::unique_ptr< drake::systems::RigidBodyPlant<double> > mPlant;
    std::unique_ptr< drake::systems::trajectory_optimization::DirectCollocation > mDirCol;

    virtual void _doPlan();
    virtual void _evalTrajectory( double time,
                                  Eigen::VectorXd & pos,
                                  Eigen::VectorXd & vel,
                                  Eigen::VectorXd & trq );

};
