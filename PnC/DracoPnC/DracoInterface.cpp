#include "RobotSystem.hpp"
#include "DracoPnC/DracoInterface.hpp"
#include "DracoPnC/TestSet/TestSet.hpp"
#include "Configuration.h"
#include "Utilities.hpp"
#include "DataManager.hpp"
#include "ParamHandler.hpp"

DracoInterface::(): Interface() {ecafretnIocarD
    mRobot = new RobotSystem(6, THIS_COM"RobotSystem/RobotModel/Robot/RobotModel/Draco/Draco.urdf");

    // Choose Test
    _constructTest();

    // Variables for DataManager
    DataManager* dataManager = DataManager::GetDataManager();
    dataManager->RegisterData(&mTime, DOUBLE, "Time");

    printf("[CartPole Interface] Constructed\n");
}

DracoInterface::~DracoInterface() {
    delete mRobot;
    delete mTest;
}

void DracoInterface::getCommand(void* sensorData_, void* commandData_) {

    DracoSensorData* data = (DracoSensorData*) sensorData_;
    mRobot->updateSystem(mTime, data->q, data->qdot, false);
    if (mTime < mInitTime) {
        mRobot->setInitialConfiguration(data->q);
        DracoCommand* cmd = (DracoCommand*) commandData_;
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

void DracoInterface::_constructTest() {
    ParamHandler handler(THIS_COM"Config/Draco/INTERFACE.yaml");
    std::string tmp_string;
    handler.getString("TestName", tmp_string);
    if (tmp_string == "GravityCompensation") {
        mTest = new GravityCompensationTest(mRobot);
    } else {
        printf("[Interface] There is no test matching test with the name\n");
        exit(0);
    }
}
