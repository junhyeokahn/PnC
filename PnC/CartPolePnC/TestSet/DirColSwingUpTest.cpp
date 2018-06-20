#include "CartPolePnC/TestSet/TestSet.hpp"
#include "CartPolePnC/PlannerSet/PlannerSet.hpp"

DirColSwingUpTest::DirColSwingUpTest(RobotSystem* robot_): Test(robot_) { // Choose Planner
    printf("[DirCol Swing Up Test] Constructed\n");
}

DirColSwingUpTest::~DirColSwingUpTest() {
}

Eigen::VectorXd DirColSwingUpTest::getTorqueInput() {
    //Eigen::VectorXd ret= Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
    //Eigen::VectorXd pos = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
    //Eigen::VectorXd vel = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
    //Eigen::VectorXd acc = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
    //Eigen::VectorXd eff = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());

    //ret = eff;

    std::cout << "asdf" << std::endl;
    //return ret;
}
