#pragma once

#include "PnC/Planner.hpp"
#include "Configuration.h"

class PreComputedPlannerParameter : public PlannerParameter
{
public:
    PreComputedPlannerParameter () : PlannerParameter() {};
    virtual ~PreComputedPlannerParameter () {};

    std::string trajectoryFile;

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
