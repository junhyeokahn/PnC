#include "RobotSystem/RobotSystem.hpp"
#include "PnC/FixedDracoPnC/FixedDracoInterface.hpp"
#include "PnC/FixedDracoPnC/TestSet/TestSet.hpp"
#include "Configuration.h"
#include "Utils/Utilities.hpp"
#include "Utils/DataManager.hpp"
#include "Utils/ParamHandler.hpp"

FixedDracoInterface::FixedDracoInterface(): Interface() {
    mRobot = new RobotSystem(0, THIS_COM"RobotSystem/RobotModel/Robot/Draco/DracoFixed.urdf");

    _constructTest();

    DataManager* dataManager = DataManager::GetDataManager();
    mJPosDes = Eigen::VectorXd::Zero(10); mJVelDes = Eigen::VectorXd::Zero(10);
    mJTrqDes = Eigen::VectorXd::Zero(10); mJPosAct = Eigen::VectorXd::Zero(10);
    mJVelAct = Eigen::VectorXd::Zero(10); mJTrqAct = Eigen::VectorXd::Zero(10);
    mMotorCurrent = Eigen::VectorXd::Zero(10); mBusVoltage = Eigen::VectorXd::Zero(10);
    mBusCurrent = Eigen::VectorXd::Zero(10); mTemperature = Eigen::VectorXd::Zero(10);

    dataManager->RegisterData(&mTime, DOUBLE, "Time");
    dataManager->RegisterData(&mJPosDes, VECT, "JPosDes", 10);
    dataManager->RegisterData(&mJVelDes, VECT, "JVelDes", 10);
    dataManager->RegisterData(&mJTrqDes, VECT, "JTrqDes", 10);
    dataManager->RegisterData(&mJPosAct, VECT, "JPosAct", 10);
    dataManager->RegisterData(&mJVelAct, VECT, "JVelAct", 10);
    dataManager->RegisterData(&mJTrqAct, VECT, "JTrqAct", 10);
    dataManager->RegisterData(&mMotorCurrent, VECT, "motorCurrent", 10);
    dataManager->RegisterData(&mBusVoltage, VECT, "BusVoltage", 10);
    dataManager->RegisterData(&mBusCurrent, VECT, "BusCurrent", 10);
    dataManager->RegisterData(&mTemperature, VECT, "Temperature", 10);
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
    } else {
        if (!mTest->isInitialized) mTest->initialize();
        DataManager::GetDataManager()->start();
        mTest->getTorqueInput(commandData_);
    }
    mTime += SERVO_RATE;

    mJPosDes = ((FixedDracoCommand*) commandData_)->q;
    mJVelDes = ((FixedDracoCommand*) commandData_)->qdot;
    mJTrqDes = ((FixedDracoCommand*) commandData_)->jtrq;
    mJPosAct = data->q;
    mJVelAct = data->qdot;
    mJTrqAct = data->jtrq;
    mMotorCurrent = data->motorCurrent;
    mBusVoltage = data->busVoltage;
    mBusCurrent = data->busCurrent;
    mTemperature = data->temperature;
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
        } else if (tmp_string == "AdmittanceTest") {
            mTest = new AdmittanceTest(mRobot);
        }
        else {
        printf("[Interface] There is no test matching test with the name\n");
        exit(0);
        }
    }catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
        exit(0);
    }
}
