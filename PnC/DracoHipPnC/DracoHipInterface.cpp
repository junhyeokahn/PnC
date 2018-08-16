#include "RobotSystem.hpp"
#include "DracoHipPnC/DracoHipInterface.hpp"
#include "DracoHipPnC/TestSet/TestSet.hpp"
#include "Configuration.h"
#include "Utilities.hpp"
#include "DataManager.hpp"
#include "ParamHandler.hpp"

DracoHipInterface::DracoHipInterface(): Interface() {
    mRobot = new RobotSystem(0, THIS_COM"RobotSystem/RobotModel/Robot/Draco/DracoFixed.urdf");

    _constructTest();

    DataManager* dataManager = DataManager::GetDataManager();
    dataManager->RegisterData(&mTime, DOUBLE, "Time");

    printf("[Draco Hip Interface] Constructed\n");
}

DracoHipInterface::~DracoHipInterface() {
    delete mRobot;
    delete mTest;
}

void DracoHipInterface::getCommand(void* sensorData_, void* commandData_) {
    DracoHipSensorData* data = (DracoHipSensorData*) sensorData_;
    mRobot->updateSystem(mTime, data->q, data->qdot, true);
    if (mTime < mInitTime) {
        mRobot->setInitialConfiguration(data->q);
        DracoHipCommand* cmd = (DracoHipCommand*) commandData_;
        cmd->q = data->q;
        cmd->qdot = Eigen::VectorXd::Zero(mRobot->getNumDofs());
        cmd->jtrq = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
        if (!mTest->isInitialized) mTest->initialize();
    } else {
        DataManager::GetDataManager()->start();
        mTest->getTorqueInput(commandData_);
    }
    mTime += SERVO_RATE;
}

void DracoHipInterface::_constructTest() {
    ParamHandler handler(THIS_COM"Config/DracoHip/INTERFACE.yaml");
    std::string tmp_string;
    handler.getString("TestName", tmp_string);
    if (tmp_string == "GravityCompensation") {
        mTest = new GravityCompensationTest(mRobot);
    } else {
        printf("[Interface] There is no test matching test with the name\n");
        exit(0);
    }
}
