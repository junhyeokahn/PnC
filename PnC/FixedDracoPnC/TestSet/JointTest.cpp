#include "FixedDracoPnC/TestSet/TestSet.hpp"
#include "RobotSystem.hpp"
#include "Utilities.hpp"
#include "DataManager.hpp"

JointTest::JointTest(RobotSystem* robot_): Test(robot_) {
    // Choose Planner

    // Choose Controller

    printf("[Joint Test] Constructed\n");
}

JointTest::~JointTest() {
}

void JointTest::getTorqueInput(void * commandData_) {
    FixedDracoCommand* cmd = (FixedDracoCommand*) commandData_;
    cmd->q = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->qdot = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->jtrq = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());

}

void JointTest::initialize() {
    mInitQ = mRobot->getInitialConfiguration();
    //Planner Initialize

    //Controller Initialize

    isInitialized = true;
}
