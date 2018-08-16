#include "DracoHipPnC/TestSet/TestSet.hpp"
#include "RobotSystem.hpp"
#include "Utilities.hpp"

GravityCompensationTest::GravityCompensationTest(RobotSystem* robot_): Test(robot_) {
    // Choose Planner

    // Choose Controller
    printf("[Whole Body Controller Test] Constructed\n");
}

GravityCompensationTest::~GravityCompensationTest() {
}

void GravityCompensationTest::getTorqueInput(void * commandData_) {
    DracoHipCommand* cmd = (DracoHipCommand*) commandData_;
    cmd->q = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->qdot = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->jtrq = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
}

void GravityCompensationTest::initialize() {
    //Planner Initialize

    //Controller Initialize

    isInitialized = true;
}
