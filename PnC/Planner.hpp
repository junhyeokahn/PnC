#pragma once

#include <iostream>
#include <Eigen/Dense>

class PlanningParameter
{
public:
    PlanningParameter () : startTime(0), endTime(0) {};
    virtual ~PlanningParameter () {};

    double startTime;
    double endTime;
private:
};

class Planner
{
public:
    Planner () : mDoPlan(true),
                 isParamSet(false) {};
    virtual ~Planner () {};

    void getPlan( double time,
                  Eigen::VectorXd & pos,
                  Eigen::VectorXd & vel,
                  Eigen::VectorXd & acc,
                  Eigen::VectorXd & eff ) {
        if (mDoPlan & isParamSet) {
            _doPlan();
            isParamSet = false;
            mDoPlan = false;
        } else if (mDoPlan & isParamSet) {
            std::cout << "Replan flag without new parameter set" << std::endl;
        };
        _evalTrajecotry( time, pos, vel, acc, eff );
    }

    void doReplan(const std::shared_ptr<PlanningParameter> & param_) {
        mDoPlan = true;
        updatePlanningParameter(param_);
    };

    void updatePlanningParameter(
            const std::shared_ptr<PlanningParameter> & param_) {
        mParam = param_;
        isParamSet = true;
    }

protected:
    bool mDoPlan;
    bool isParamSet;
    std::shared_ptr<PlanningParameter> mParam;
    // Do Planning with PlanningParameter.
    virtual void _doPlan() = 0;
    virtual void _evalTrajecotry( double time, Eigen::VectorXd & pos,
                                  Eigen::VectorXd & vel, Eigen::VectorXd acc,
                                  Eigen::VectorXd &eff )= 0;
};
