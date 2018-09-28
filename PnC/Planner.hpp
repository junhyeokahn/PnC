#pragma once

#include <iostream>
#include <memory>

#include <Eigen/Dense>

#include "Configuration.h"
#include "Utils/Utilities.hpp"
#include "Utils/Clock.hpp"

class PlannerParameter
{
public:
    PlannerParameter () : startTime(0), endTime(0) {};
    virtual ~PlannerParameter () {};

    double startTime;
    double endTime;
    int numState;
    int numInput;
private:
};

class Planner
{
public:
    Planner () : mDoPlan(true), isParamSet(false)
    {
        mClock = new Clock();
    };
    virtual ~Planner () {
        delete mClock;
    };

    void getPlan( double time,
                  Eigen::VectorXd & pos,
                  Eigen::VectorXd & vel,
                  Eigen::VectorXd & trq ) {
        if (mDoPlan & isParamSet) {
            mClock->start();
            _doPlan();
            std::cout << "Computation Time For Planning : " << mClock->stop() <<
                " (ms)" << std::endl;
            isParamSet = false;
            mDoPlan = false;
        } else if (mDoPlan & isParamSet) {
            std::cout << "Replan flag without new parameter set" << std::endl;
            exit(0);
        };
        _evalTrajectory( time, pos, vel, trq );
    }

    void doReplan(const std::shared_ptr<PlannerParameter> & param_) {
        mDoPlan = true;
        updatePlanningParameter(param_);
    };

    void updatePlanningParameter(
            const std::shared_ptr<PlannerParameter> & param_) {
        mParam = param_;
        isParamSet = true;
    }

protected:
    bool mDoPlan;
    bool isParamSet;
    std::shared_ptr<PlannerParameter> mParam;
    // Do Planning with PlannerParameter.
    virtual void _doPlan() = 0;
    virtual void _evalTrajectory( double time, Eigen::VectorXd & pos,
                                               Eigen::VectorXd & vel,
                                               Eigen::VectorXd & trq ) = 0;
    Clock * mClock;
};
