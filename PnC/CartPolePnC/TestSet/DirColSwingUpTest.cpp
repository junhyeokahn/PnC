#include "CartPolePnC/TestSet/DirColSwingUpTest.hpp"
#include "RobotSystem.hpp"
#include "Utilities.hpp"
#include "Configuration.h"

DirColSwingUpTest::DirColSwingUpTest(RobotSystem* robot_): Test(robot_) { // Choose Planner
    mTree = std::make_unique< RigidBodyTree<double> >();
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
            THIS_COM"Simulator/SimulationModel/RobotModel/CartPole/CartPole.urdf",
            drake::multibody::joints::kFixed, mTree.get());
    mPlant = std::make_unique< drake::systems::RigidBodyPlant<double> >(
            std::move(mTree), 0.0);
    printf("[DirCol Swing Up Test] Constructed\n");
}

DirColSwingUpTest::~DirColSwingUpTest() {
}

Eigen::VectorXd DirColSwingUpTest::getTorqueInput() {
    Eigen::VectorXd ret = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
    ret = mRobot->getCoriolisGravity();

    return ret;
}

void DirColSwingUpTest::initialize() {
    // Initialize Planner
    if (!mIsInitialized) {
        mIsInitialized = true;
    }
    // Initialize Controller
}
