#pragma once

#include "PnC/EnvInterface.hpp"
#include "PnC/Test.hpp"

class FixedAtlasSensorData {
   public:
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    //Sensor_Data
};

class FixedAtlasCommand {
   public:
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd jtrq;
};

class FixedAtlasInterface : public EnvInterface {
   protected:
    int count_;

    void _ParameterSetting();

   public:
    FixedAtlasInterface();
    virtual ~FixedAtlasInterface();
    virtual void getCommand(void* _sensor_data, void* _command_data);
};
