#include "FixedDracoPnC/TestSet/TestSet.hpp"
#include "RobotSystem.hpp"
#include "Utilities.hpp"
#include "DataManager.hpp"

InvKinTest::InvKinTest(RobotSystem* robot_): Test(robot_) {
    // Choose Planner

    // Choose Controller

    printf("[InvKin Test] Constructed\n");
}

InvKinTest::~InvKinTest() {
}

void InvKinTest::getTorqueInput(void * commandData_) {
    FixedDracoCommand* cmd = (FixedDracoCommand*) commandData_;
    cmd->q = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->qdot = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->jtrq = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());

}

void InvKinTest::initialize() {
    mTestInitQ = mRobot->getInitialConfiguration();
    //Planner Initialize

    //Controller Initialize

    isInitialized = true;
}
