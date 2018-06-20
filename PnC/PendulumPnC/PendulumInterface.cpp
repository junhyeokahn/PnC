#include "RobotSystem.hpp"
#include "PendulumPnC/PendulumInterface.hpp"
#include "PendulumPnC/TestSet/TestSet.hpp"
#include "Configuration.h"
#include "Utilities.hpp"
#include "Utils/DataManager.hpp"

PendulumInterface::PendulumInterface(): Interface() {
    mRobot = new RobotSystem(0, THIS_COM"Simulator/SimulationModel/RobotModel/Pendulum/Pendulum.urdf");

    // Choose Test
    _constructTest();

    // Variables for DataManager
    DataManager* dataManager = DataManager::GetDataManager();
    dataManager->RegisterData(&mTime, DOUBLE, "Time");

    printf("[Pendulum Interface] Constructed\n");
}

PendulumInterface::~PendulumInterface() {
    delete mRobot;
    delete mTest;
}

Eigen::VectorXd PendulumInterface::getCommand(void* sensorData_) {
    PendulumSensorData* data = ((PendulumSensorData*) sensorData_);
    mRobot->updateSystem(mTime, data->q, data->qdot);
    mTime += SERVO_RATE;

    if (mTime < mInitTime) {
        mRobot->setInitialConfiguration(data->q);
        return Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
    } else {
        DataManager::GetDataManager()->start();
        return mTest->getTorqueInput();
    }
}

void PendulumInterface::_constructTest() {
    mTest = new GravityCompensationTest(mRobot);
}
