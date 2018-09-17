#include "RobotSystem/RobotSystem.hpp"
#include "PnC/CartPolePnC/CartPoleInterface.hpp"
#include "PnC/CartPolePnC/TestSet/TestSet.hpp"
#include "Configuration.h"
#include "Utils/Utilities.hpp"
#include "Utils/DataManager.hpp"
#include "Utils/ParamHandler.hpp"

CartPoleInterface::CartPoleInterface(): Interface() {
    mRobot = new RobotSystem(1, THIS_COM"RobotSystem/RobotModel/Robot/CartPole/CartPole.urdf");

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

void CartPoleInterface::getCommand(void* sensorData_, void* commandData_) {

    CartPoleSensorData* data = (CartPoleSensorData*) sensorData_;
    mRobot->updateSystem(mTime, data->q, data->qdot, false);
    if (mTime < mInitTime) {
        mRobot->setInitialConfiguration(data->q);
        CartPoleCommand* cmd = (CartPoleCommand*) commandData_;
        cmd->q = data->q;
        cmd->qdot = Eigen::VectorXd::Zero(mRobot->getNumDofs());
        cmd->jtrq = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
    } else {
        if (!mTest->isInitialized)
            mTest->initialize();
        DataManager::GetDataManager()->start();
        mTest->getTorqueInput(commandData_);
    }
    mTime += SERVO_RATE;
}

void CartPoleInterface::_constructTest() {
    ParamHandler handler(THIS_COM"Config/CartPole/INTERFACE.yaml");
    std::string tmp_string;
    handler.getString("TestName", tmp_string);
    if (tmp_string == "DirColSwingUpTest") {
        mTest = new DirColSwingUpTest(mRobot);
    } else {
        printf("[Interface] There is no test matching with the name\n");
        exit(0);
    }
}
