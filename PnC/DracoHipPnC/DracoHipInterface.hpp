#pragma once

#include "Interface.hpp"

class DracoHipSensorData
{
public:
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd jtrq;
};

class DracoHipCommand
{
public:
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd jtrq;
};


class DracoHipInterface: public Interface
{
protected:
    void _constructTest();

public:
    DracoHipInterface();
    virtual ~DracoHipInterface();
    virtual void getCommand(void* sensorData_, void* commandData_);
};
