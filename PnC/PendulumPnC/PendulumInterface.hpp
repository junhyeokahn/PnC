#pragma once

#include "Interface.hpp"

class PendulumSensorData
{
public:
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
};

class PendulumInterface: public Interface
{
protected:
    void _constructTest();

public:
    PendulumInterface();
    virtual ~PendulumInterface();
    virtual Eigen::VectorXd getCommand(void* sensorData_);
};
