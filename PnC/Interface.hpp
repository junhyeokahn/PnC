#pragma once

#include <Eigen/Dense>
#include <memory>

class Test;
class RobotSystem;

class Interface
{
protected:
    Test* test_;
    RobotSystem* robot_;
    int count_;
    double running_time_;

public:
    Interface() {
        count_ = 0;
        running_time_ = 0.;
    }
    virtual ~Interface() {};

    // Get Command through Test
    virtual void getCommand(void* _sensor_data, void* _command_data) = 0;
};
