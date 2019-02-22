#pragma once

#include <Eigen/Dense>
#include <memory>

class Test;
class RobotSystem;

class EnvInterface {
   protected:
    Test* test_;
    RobotSystem* robot_;
    int count_;
    double running_time_;

   public:
    EnvInterface() {
        count_ = 0;
        running_time_ = 0.;
    }
    virtual ~EnvInterface(){};

    // Get Command through Test
    virtual void getCommand(void* _sensor_data, void* _command_data) = 0;
};
