#include "RobotSystem/RobotSystem.hpp"
#include "PnC/DracoPnC/DracoInterface.hpp"
#include "PnC/DracoPnC/TestSet/TestSet.hpp"
#include "Configuration.h"
#include "Utils/Utilities.hpp"
#include "Utils/DataManager.hpp"
#include "Utils/ParamHandler.hpp"

#include <drake/common/find_resource.h>
#include <drake/multibody/ik_options.h>
#include <drake/multibody/joints/floating_base_types.h>
#include <drake/multibody/parsers/urdf_parser.h>
#include <drake/multibody/rigid_body_constraint.h>
#include <drake/multibody/rigid_body_ik.h>
#include <drake/multibody/rigid_body_tree.h>
#include "drake/math/eigen_sparse_triplet.h"

DracoInterface::DracoInterface(): Interface() {
    mRobot = new RobotSystem(6, THIS_COM"RobotSystem/RobotModel/Robot/Draco/Draco.urdf");

    _constructTest();

    DataManager* dataManager = DataManager::GetDataManager();
    dataManager->RegisterData(&mTime, DOUBLE, "Time");
    mTrqCmd = Eigen::VectorXd::Zero(10);
    dataManager->RegisterData(&mTrqCmd, VECT, "TorqueCommand", 10);

    printf("[Draco Interface] Constructed\n");
}

DracoInterface::~DracoInterface() {
    delete mRobot;
    delete mTest;
}

void DracoInterface::getCommand(void* sensorData_, void* commandData_) {
    DracoSensorData* data = (DracoSensorData*) sensorData_;
    mRobot->updateSystem(mTime, data->q, data->qdot, true);
    if (mTime < mInitTime) {
        mRobot->setInitialConfiguration(data->q);
        DracoCommand* cmd = (DracoCommand*) commandData_;
        cmd->q = data->q;
        cmd->qdot = Eigen::VectorXd::Zero(mRobot->getNumDofs());
        cmd->jtrq = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
        //if (!mTest->isInitialized) mTest->initialize();
    } else {
        if (!mTest->isInitialized) mTest->initialize();
        DataManager::GetDataManager()->start();
        mTest->getTorqueInput(commandData_);
        mTrqCmd = ((DracoCommand*) commandData_)->jtrq.tail(10);
    }
    mTime += SERVO_RATE;
}

void DracoInterface::_constructTest() {
    try {
        YAML::Node cfg = YAML::LoadFile(THIS_COM"Config/Draco/INTERFACE.yaml");
        std::string tmp_string;
        //std::cout << readParameter<std::string>(cfg, "TestName") << std::endl;
        myUtils::readParameter(cfg, "TestName", tmp_string);
        if (tmp_string == "WholeBodyControllerTest") {
            mTest = new WholeBodyControllerTest(mRobot);
        } else if (tmp_string == "SymExpValidationTest") {
            mTest = new SymExpValidationTest(mRobot);
        } else if (tmp_string == "CentroidKinematicOptimizationTest") {
            mTest = new CentroidKinematicOptimizationTest(mRobot);
        } else if (tmp_string == "InvKinTest") {
            mTest = new InvKinTest(mRobot);
        } else if (tmp_string == "AdmittanceTest") {
            mTest = new AdmittanceTest(mRobot);
        } else if (tmp_string == "SteppingTest") {
            mTest = new SteppingTest(mRobot);
        } else {
            printf("[Interface] There is no test matching test with the name\n");
            exit(0);
        }
    }catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
        exit(0);
    }
}
