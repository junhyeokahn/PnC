#pragma once

#include "Planner.hpp"
#include "RobotSystem.hpp"

class KinematicPlannerParameter : public PlannerParameter
{
public:
    KinematicPlannerParameter () : PlannerParameter() {};
    virtual ~KinematicPlannerParameter () {};

private:
    /* data */
};
class KinematicsPlanner : public Planner
{
public:
    KinematicsPlanner ();
    virtual ~KinematicsPlanner ();

private:
    virtual void _doPlan();
    virtual void _evalTrajectory( double time, Eigen::VectorXd& pos,
                                               Eigen::VectorXd& vel,
                                               Eigen::VectorXd& trq);
};
