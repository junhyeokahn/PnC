#pragma once

#include <iostream>
#include <memory>

#include <Eigen/Dense>

#include "Configuration.h"

class Planner {
   public:
    Planner(){};
    virtual ~Planner(){};

    virtual void DoPlan(bool b_use_previous_solution = false) = 0;
    virtual void EvalTrajectory(double time, Eigen::VectorXd& s,
                                Eigen::VectorXd& sdot, Eigen::VectorXd& u) = 0;

   protected:
};
