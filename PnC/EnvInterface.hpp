#pragma once

#include <Eigen/Dense>
#include <memory>
#include <PnC/InterruptLogic.hpp>

class Test;
class ControlArchitecture;
class RobotSystem;

class EnvInterface {
   protected:
    Test* test_;
    ControlArchitecture* control_architecture_;
    RobotSystem* robot_;
    int count_;
    double running_time_;

   public:
    InterruptLogic* interrupt_;

    EnvInterface() {
        count_ = 0;
        running_time_ = 0.;
    }
    virtual ~EnvInterface(){};

    // Get Command through Test
    virtual void getCommand(void* _sensor_data, void* _command_data) = 0;
};
