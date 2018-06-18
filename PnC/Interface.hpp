#ifndef INTERFACE_H
#define INTERFACE_H

#include <Eigen/Dense>

class Test;
class RobotSystem;

class Interface
{
protected:
    Test* mTest;
    RobotSystem* mRobot;
    double mTime;
    double mInitTime;

public:
    Interface(): mInitTime(0.002), mTime(0.0) {};
    virtual ~Interface() {};

    // Get Command through Test
    virtual Eigen::VectorXd getCommand(void* sensorData_) = 0;
};

#endif /* INTERFACE_H */
