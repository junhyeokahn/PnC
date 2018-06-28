#pragma once

#include "Interface.hpp"

class DracoSensorData
{
public:
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd jtrq;
};

class DracoCommand
{
public:
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd jtrq;
};


class DracoInterface: public Interface
{
protected:
    void _constructTest();

public:
    DracoInterface();
    virtual ~DracoInterface();
    virtual void getCommand(void* sensorData_, void* commandData_);
};
