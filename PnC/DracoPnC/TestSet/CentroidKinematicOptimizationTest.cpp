#include "PnC/DracoPnC/TestSet/TestSet.hpp"
#include "RobotSystem/RobotSystem.hpp"
#include "Utils/Utilities.hpp"
#include "Utils/DataManager.hpp"
#include "PnC/WBC/WBLC/WBLC.hpp"
#include "PnC/WBC/WBLC/WBLCContact.hpp"

CentroidKinematicOptimizationTest::CentroidKinematicOptimizationTest(RobotSystem* robot_): Test(robot_) {
    // Choose Planner

    // Choose Controller
    std::vector<bool> act_list;
    act_list.resize(mRobot->getNumDofs(), true);
    for(int i(0); i < mRobot->getNumVirtualDofs(); ++i)
        act_list[i] = false;
    mWBLC = new WBLC(act_list);
    mWBLCExtraData= new WBLC_ExtraData();
    mJointTask = new Task(mRobot, TaskType::JOINT);
    mTaskList.push_back(mJointTask);
    mRfContact = new WBLCContact(mRobot, "rAnkle", 0.7);
    mLfContact = new WBLCContact(mRobot, "lAnkle", 0.7);
    mContactList.clear();
    mContactList.push_back(mRfContact);
    mContactList.push_back(mLfContact);
    printf("[Centroid Kinematic Optimization Test] Constructed\n");
}

CentroidKinematicOptimizationTest::~CentroidKinematicOptimizationTest() {
    delete mWBLC;
    delete mWBLCExtraData;
    delete mJointTask;
    delete mRfContact;
    delete mLfContact;
}

void CentroidKinematicOptimizationTest::getTorqueInput(void * commandData_) {

    DracoCommand* cmd = (DracoCommand*) commandData_;
    cmd->q = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->qdot = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->jtrq = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());

    // Planner


    // Controller
    _updateContact();
    _updateTask();
    _WBLCpreProcess();
    mWBLC->MakeTorque(mTaskList, mContactList, cmd->jtrq, mWBLCExtraData);
    _WBLCpostProcess();
}

void CentroidKinematicOptimizationTest::initialize() {
    mTestInitQ = mRobot->getQ();
    mTestInitTime = mRobot->getTime();

    //Planner Initialize
    try {
        YAML::Node test_cfg = YAML::LoadFile(THIS_COM"Config/Draco/TEST/CENTROID_KINEMATIC_TEST.yaml");
        YAML::Node initial_cfg = test_cfg["initial_configuration"];
        myUtils::readParameter(initial_cfg, "transition_time", mInterpolationDuration);
        myUtils::readParameter(initial_cfg, "joint_position", mInterpolationPosition);
    }catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }
    double ini[30]; double fin[30]; double **middle_pt;
    for (int i = 0; i < 30; ++i) {
        ini[i] = 0.0; fin[i] = 0.0;
    }
    for (int i = 0; i < 10; ++i) {
        ini[i] = mTestInitQ[i]; fin[i] = mInterpolationPosition[i];
    }
    mSpline.SetParam(ini, fin, middle_pt, mInterpolationDuration);

    //Controller Initialize

    isInitialized = true;
}

void CentroidKinematicOptimizationTest::_updateTask() {
    Eigen::VectorXd pos = (mRobot->getInitialConfiguration()).tail(mRobot->getNumActuatedDofs());
    Eigen::VectorXd vel = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
    Eigen::VectorXd acc = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
    mTaskList[0]->updateTaskSpec(pos, vel, acc);
}


void CentroidKinematicOptimizationTest::_updateContact() {
    for (int i = 0; i < mContactList.size(); ++i)
        mContactList[i]->updateWBLCContactSpec();
}

void CentroidKinematicOptimizationTest::_WBLCpostProcess() {
    for(int i(0); i<mTaskList.size(); ++i)
        mTaskList[i]->unsetTaskSpec();
    for(int i(0); i<mContactList.size(); ++i)
        mContactList[i]->unsetWBLCContactSpec();
}

void CentroidKinematicOptimizationTest::_WBLCpreProcess() {
    // dynamic property setting
    mWBLC->UpdateSetting(mRobot->getMassMatrix(),
                         mRobot->getInvMassMatrix(),
                         mRobot->getCoriolis(),
                         mRobot->getGravity());
    // cost setting
    int taskDim(0);
    int contactDim(0);
    for (int i = 0; i < mTaskList.size(); ++i)
        taskDim += mTaskList[i]->getDims();
    for (int i = 0; i < mContactList.size(); ++i)
        contactDim += 6;
    mWBLCExtraData->cost_weight = Eigen::VectorXd::Zero(taskDim + contactDim);

    for(int i(0); i<taskDim; ++i) {
        mWBLCExtraData->cost_weight[i] = 10000.0;
    }
    for(int i(0); i<contactDim; ++i){
        mWBLCExtraData->cost_weight[taskDim + i] = 1.0;
    }
    mWBLCExtraData->cost_weight[taskDim + 5] = 0.001; // vertical reaction
    mWBLCExtraData->cost_weight[taskDim + 11] = 0.001; // vertical reaction
}

