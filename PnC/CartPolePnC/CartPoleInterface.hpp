#pragma once

#include "Interface.hpp"

class CartPoleSensorData
{
public:
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
};

class CartPoleInterface: public Interface
{
protected:
    void _constructTest();

public:
    CartPoleInterface();
    virtual ~CartPoleInterface();
    virtual Eigen::VectorXd getCommand(void* sensorData_);
};
