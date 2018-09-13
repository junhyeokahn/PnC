#include "RobotSystem.hpp"
#include "FixedDracoPnC/FixedDracoInterface.hpp"
#include "FixedDracoPnC/TestSet/TestSet.hpp"
#include "Configuration.h"
#include "Utilities.hpp"
#include "DataManager.hpp"
#include "ParamHandler.hpp"

FixedDracoInterface::FixedDracoInterface(): Interface() {
    mRobot = new RobotSystem(0, THIS_COM"RobotSystem/RobotModel/Robot/Draco/DracoFixed.urdf");

    _constructTest();

    DataManager* dataManager = DataManager::GetDataManager();
    dataManager->RegisterData(&mTime, DOUBLE, "Time");

    printf("[Fixed Draco Interface] Constructed\n");
}

FixedDracoInterface::~FixedDracoInterface() {
    delete mRobot;
    delete mTest;
}

void FixedDracoInterface::getCommand(void* sensorData_, void* commandData_) {
    FixedDracoSensorData* data = (FixedDracoSensorData*) sensorData_;
    mRobot->updateSystem(mTime, data->q, data->qdot, false);
    if (mTime < mInitTime) {
        mRobot->setInitialConfiguration(data->q);
        FixedDracoCommand* cmd = (FixedDracoCommand*) commandData_;
        cmd->q = data->q;
        cmd->qdot = Eigen::VectorXd::Zero(mRobot->getNumDofs());
        cmd->jtrq = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
        //if (!mTest->isInitialized) mTest->initialize();
    } else {
        if (!mTest->isInitialized) mTest->initialize();
        DataManager::GetDataManager()->start();
        mTest->getTorqueInput(commandData_);
    }
    mTime += SERVO_RATE;
}

void FixedDracoInterface::_constructTest() {
    try {
        YAML::Node cfg = YAML::LoadFile(THIS_COM"Config/FixedDraco/INTERFACE.yaml");
        std::string tmp_string;
        //std::cout << readParameter<std::string>(cfg, "TestName") << std::endl;
        myUtils::readParameter(cfg, "TestName", tmp_string);
        if (tmp_string == "OSCTest") {
            mTest = new OSCTest(mRobot);
        } else if (tmp_string == "InvKinTest") {
            mTest = new InvKinTest(mRobot);
        } else if (tmp_string == "JointTest") {
            mTest = new JointTest(mRobot);
        } else {
        printf("[Interface] There is no test matching test with the name\n");
        exit(0);
        }
    }catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
        exit(0);
    }
}
