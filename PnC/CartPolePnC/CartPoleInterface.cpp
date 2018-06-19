#include "RobotSystem.hpp"
#include "CartPolePnC/CartPoleInterface.hpp"
#include "CartPolePnC/TestSet/TestSet.hpp"
#include "Configuration.h"
#include "Utilities.hpp"
#include "DataManager.hpp"
#include "ParamHandler.hpp"

CartPoleInterface::CartPoleInterface(): Interface() {
    mRobot = new RobotSystem(1, THIS_COM"Simulator/SimulationModel/RobotModel/CartPole/CartPole.urdf");

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
    mRobot->updateSystem(mTime, data->q, data->qdot, false);
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
    ParamHandler handler(THIS_COM"Config/CartPole/INTERFACE.yaml");
    std::string tmp_string;
    handler.getString("TestName", tmp_string);
    if (tmp_string == "GravityCompensationTest") {
        mTest = new GravityCompensationTest(mRobot);
    } else if (tmp_string == "DirColSwingUpTest") {
        mTest = new DirColSwingUpTest(mRobot);
    } else {
        printf("[Interface] There is no test matching with the name\n");
        exit(0);
    }
}
