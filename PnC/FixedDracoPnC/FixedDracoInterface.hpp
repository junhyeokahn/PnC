#pragma once

#include "PnC/EnvInterface.hpp"
#include "PnC/Test.hpp"

class FixedDracoSensorData {
   public:
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
};

class FixedDracoCommand {
   public:
    Eigen::VectorXd jtrq;
};

class FixedDracoInterface : public EnvInterface {
   protected:
    int count_;

    void _ParameterSetting();

    Test* test_;

   public:
    FixedDracoInterface();
    virtual ~FixedDracoInterface();
    virtual void getCommand(void* _sensor_data, void* _command_data);
};
