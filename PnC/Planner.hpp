#pragma once
#include <Eigen/Dense>

class PlanningParameter
{
public:
    PlanningParameter (double startTime_, double endTime_) :
        startTime(startTime_), endTime(endTime_) {};
    virtual ~PlanningParameter ();

    double startTime;
    double endTime;
private:
    /* data */
};

class Planner
{
public:
    Planner (std::shared_ptr<PlanningParameter> param_) : mDoReplan(true),
                                                          param(param_) {};
    virtual ~Planner () {};

    void getPlan( double time,
                  Eigen::VectorXd & pos,
                  Eigen::VectorXd & vel,
                  Eigen::VectorXd & acc,
                  Eigen::VectorXd & eff ) {
        if (mDoReplan) _doPlan();
        _evalTrajecotry( time, pos, vel, acc, eff );
    }
    void doReplan() { mDoReplan = true; };

    std::shared_ptr<PlanningParameter> param;

private:
    bool mDoReplan;
    virtual void _doPlan() = 0;
    virtual void _evalTrajecotry( double time, Eigen::VectorXd & pos,
                                  Eigen::VectorXd & vel, Eigen::VectorXd acc,
                                  Eigen::VectorXd &eff )= 0;
};
