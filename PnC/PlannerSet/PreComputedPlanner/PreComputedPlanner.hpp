#pragma once

#include "Planner.hpp"
#include "Configuration.h"

class PreComputedPlannerParameter : public PlannerParameter
{
public:
    PreComputedPlannerParameter () : PlannerParameter() {};
    virtual ~PreComputedPlannerParameter () {};

private:
    /* data */
};

class PreComputedPlanner : public Planner
{
public:
    PreComputedPlanner ();
    virtual ~PreComputedPlanner ();

private:
    /* data */
    virtual void _doPlan();
    virtual void _evalTrajectory( double time,
                                  Eigen::VectorXd & pos,
                                  Eigen::VectorXd & vel,
                                  Eigen::VectorXd & trq);
    std::shared_ptr<PreComputedPlannerParameter> mPreComputedParam;
};
