#pragma once

#include "PnC/EnvInterface.hpp"
#include "PnC/Test.hpp"

class AtlasSensorData {
   public:
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
};

class AtlasCommand {
   public:
    Eigen::VectorXd jtrq;
};

class AtlasInterface : public EnvInterface {
   protected:
    int count_;

    void _ParameterSetting();

    Test* test_; //why restate this? already exist in EnvInterface?

   public:
    AtlasInterface();
    virtual ~AtlasInterface();
    virtual void getCommand(void* _sensor_data, void* _command_data);
};
