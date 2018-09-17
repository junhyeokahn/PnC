#pragma once

#include "PnC/Interface.hpp"

class CartPoleSensorData
{
public:
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
};

class CartPoleCommand
{
public:
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd jtrq;
};


class CartPoleInterface: public Interface
{
protected:
    void _constructTest();

public:
    CartPoleInterface();
    virtual ~CartPoleInterface();
    virtual void getCommand(void* sensorData_, void* commandData_);
};
