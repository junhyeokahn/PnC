#ifndef INTERFACE_H
#define INTERFACE_H

#include <Eigen/Dense>
#include <memory>

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
    Interface(): mInitTime(0.007), mTime(0.0) {};
    virtual ~Interface() {};

    // Get Command through Test
    virtual void getCommand(void* sensorData_, void* commandData_) = 0;
};

#endif /* INTERFACE_H */
