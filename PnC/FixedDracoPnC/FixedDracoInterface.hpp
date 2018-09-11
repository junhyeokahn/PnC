#pragma once

#include "Interface.hpp"

class FixedDracoSensorData
{
public:
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd jtrq;
};

class FixedDracoCommand
{
public:
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd jtrq;
};


class FixedDracoInterface: public Interface
{
protected:
    void _constructTest();

public:
    FixedDracoInterface();
    virtual ~FixedDracoInterface();
    virtual void getCommand(void* sensorData_, void* commandData_);
};
