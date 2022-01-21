#pragma once

#include "pnc/robot_system/robot_system.hpp"
#include "pnc/humanoid_sensor_data.h"

//TODO convert includes to forward declarations

class HumanoidStateEstimator{
public:
    HumanoidStateEstimator(RobotSystem *robot){robot_ = robot;};
    virtual ~HumanoidStateEstimator() = default;

    virtual void initialize(HumanoidSensorData *sensorData) = 0;
    virtual void update(HumanoidSensorData *sensorData) = 0;

//protected:
    RobotSystem *robot_;

    bool b_first_visit_;
};