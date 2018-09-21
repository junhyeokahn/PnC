#pragma once

#include "PnC/Interface.hpp"

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

    Eigen::VectorXd mJPosDes;
    Eigen::VectorXd mJVelDes;
    Eigen::VectorXd mJTrqDes;
    Eigen::VectorXd mJPosAct;
    Eigen::VectorXd mJVelAct;
    Eigen::VectorXd mJTrqAct;

public:
    FixedDracoInterface();
    virtual ~FixedDracoInterface();
    virtual void getCommand(void* sensorData_, void* commandData_);
};
