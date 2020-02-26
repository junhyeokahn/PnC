#pragma once
#include <utility>

#include <Configuration.h>
#include <Utils/General/Clock.hpp>
#include <Utils/IO/IOUtilities.hpp>

class RobotSystem;

class ScorpioStateProvider {
   public:
    static ScorpioStateProvider* getStateProvider(RobotSystem* _robot);
    ~ScorpioStateProvider() {}

    void saveCurrentData();

    Clock clock;

    double curr_time;

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;

    Eigen::VectorXd jpos_ini;

    int phase_copy;

    bool is_moving;
    bool is_grasping;

   private:
    ScorpioStateProvider(RobotSystem* _robot);
    RobotSystem* robot_;
};
