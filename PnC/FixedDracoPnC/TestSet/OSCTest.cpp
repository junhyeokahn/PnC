#include "FixedDracoPnC/TestSet/TestSet.hpp"
#include "RobotSystem.hpp"
#include "Utilities.hpp"
#include "DataManager.hpp"

OSCTest::OSCTest(RobotSystem* robot_): Test(robot_) {
    // Choose Planner

    // Choose Controller

    printf("[OSC Test] Constructed\n");
}

OSCTest::~OSCTest() {
}

void OSCTest::getTorqueInput(void * commandData_) {
    FixedDracoCommand* cmd = (FixedDracoCommand*) commandData_;
    cmd->q = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->qdot = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->jtrq = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());

}

void OSCTest::initialize() {
    mInitQ = mRobot->getInitialConfiguration();
    //Planner Initialize

    //Controller Initialize

    isInitialized = true;
}
