#include "RobotSystem.hpp"
#include "CartPolePnC/CartPoleInterface.hpp"
#include "CartPolePnC/TestSet/TestSet.hpp"
#include "Configuration.h"
#include "Utilities.hpp"
#include "Utils/DataManager.hpp"

CartPoleInterface::CartPoleInterface(): Interface() {
    mRobot = new RobotSystem(0, THIS_COM"Simulator/SimulationModel/RobotModel/CartPole/CartPole.urdf");

    // Choose Test
    _constructTest();

    // Variables for DataManager
    DataManager* dataManager = DataManager::GetDataManager();
    dataManager->RegisterData(&mTime, DOUBLE, "Time");

    printf("[CartPole Interface] Constructed\n");
}

CartPoleInterface::~CartPoleInterface() {
    delete mRobot;
    delete mTest;
}

Eigen::VectorXd CartPoleInterface::getCommand(void* sensorData_) {
    CartPoleSensorData* data = ((CartPoleSensorData*) sensorData_);
    mRobot->updateSystem(mTime, data->q, data->qdot);
    mTime += SERVO_RATE;

    if (mTime < mInitTime) {
        mRobot->setInitialConfiguration(data->q);
        return Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
    } else {
        mTest->initialize();
        DataManager::GetDataManager()->start();
        return mTest->getTorqueInput();
    }
}

void CartPoleInterface::_constructTest() {
    mTest = new GravityCompensationTest(mRobot);
}
