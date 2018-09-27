#pragma once

#include "PnC/Planner.hpp"
#include "Configuration.h"

class PrePlannedCentroidPlannerParameter : public PlannerParameter
{
public:
    PrePlannedCentroidPlannerParameter () : PlannerParameter() {};
    virtual ~PrePlannedCentroidPlannerParameter () {};

    std::string trajectoryFile;

private:
    /* data */
};

class PrePlannedCentroidPlanner : public Planner
{
public:
    PrePlannedCentroidPlanner ();
    virtual ~PrePlannedCentroidPlanner ();

private:
    /* data */
    virtual void _doPlan();
    virtual void _evalTrajectory( double time,
                                  Eigen::VectorXd & pos,
                                  Eigen::VectorXd & vel,
                                  Eigen::VectorXd & trq);
    std::shared_ptr<PrePlannedCentroidPlannerParameter> mPreComputedParam;
};
