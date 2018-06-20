#include "PendulumPnC/TestSet/GravityCompensationTest.hpp"
#include "RobotSystem.hpp"
#include "Utilities.hpp"

GravityCompensationTest::GravityCompensationTest(RobotSystem* robot_): Test(robot_) { // Choose Planner
    printf("[Gravity Compensation Test] Constructed\n");
}

GravityCompensationTest::~GravityCompensationTest() {
}

Eigen::VectorXd GravityCompensationTest::getTorqueInput() {
    Eigen::VectorXd ret = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
    ret = mRobot->getCoriolisGravity();

    return ret;
}
