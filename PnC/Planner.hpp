#pragma once

#include <iostream>
#include <Eigen/Dense>
#include "Configuration.h"
#include "Utilities.hpp"

class PlanningParameter
{
public:
    PlanningParameter () : startTime(0), endTime(0) {};
    virtual ~PlanningParameter () {};

    double startTime;
    double endTime;
    int numState;
    int numInput;
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
                  Eigen::VectorXd & eff ) {
        if (mDoPlan & isParamSet) {
            _doPlan();
            isParamSet = false;
            mDoPlan = false;
        } else if (mDoPlan & isParamSet) {
            std::cout << "Replan flag without new parameter set" << std::endl;
        };
        _evalTrajectory( time, pos, vel, eff );
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

protected:
    bool mDoPlan;
    bool isParamSet;
    std::shared_ptr<PlanningParameter> mParam;
    // Do Planning with PlanningParameter.
    virtual void _doPlan() = 0;
    virtual void _evalTrajectory( double time, Eigen::VectorXd & pos,
                                               Eigen::VectorXd & vel,
                                               Eigen::VectorXd & eff ) = 0;
};
