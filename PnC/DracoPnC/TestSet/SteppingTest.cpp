#include "PnC/DracoPnC/TestSet/TestSet.hpp"
#include "RobotSystem/RobotSystem.hpp"
#include "PnC/WBC/Task.hpp"
#include "PnC/WBC/WBLC/WBLC.hpp"
#include "PnC/WBC/WBLC/WBLCContact.hpp"
#include "Utils/Utilities.hpp"

SteppingTest::SteppingTest(RobotSystem* robot_) : Test(robot_),
                                                  mCurrentPhase(0),
                                                  mDoUpdateContactListAndTaskList(true) {
    try {
        YAML::Node test_cfg =
            YAML::LoadFile(THIS_COM"Config/Draco/TEST/STEPPING_TEST.yaml");
        YAML::Node control_cfg = test_cfg["control_configuration"];
        myUtils::readParameter(control_cfg, "centroid_task_kp", mCentroidTaskKp);
        myUtils::readParameter(control_cfg, "centroid_task_kd", mCentroidTaskKd);
        myUtils::readParameter(control_cfg, "joint_task_kp", mJointTaskKp);
        myUtils::readParameter(control_cfg, "joint_task_kd", mJointTaskKd);
        myUtils::readParameter(control_cfg, "com_rpy_task_kp", mCoMRPYTaskKp);
        myUtils::readParameter(control_cfg, "com_rpy_task_kd", mCoMRPYTaskKd);
        YAML::Node phase_cfg = test_cfg["phase_configuration"];
        myUtils::readParameter(phase_cfg, "num_phase", mNumPhase);
        myUtils::readParameter(phase_cfg, "phase_duration", mPhaseTimeHorizon);
        myUtils::readParameter(phase_cfg, "wait_time_between_phase", mWaitTimeBtwPhase);

    }catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }

    // Choose Controller
    std::vector<bool> act_list;
    act_list.resize(mRobot->getNumDofs(), true);
    for(int i(0); i < mRobot->getNumVirtualDofs(); ++i)
        act_list[i] = false;
    mWBLC = new WBLC(act_list);
    mWBLCExtraData= new WBLC_ExtraData();
    mRfContact = new WBLCContact(mRobot, "rAnkle", 0.7);
    mLfContact = new WBLCContact(mRobot, "lAnkle", 0.7);
    mFixedBodyContact = new WBLCContact(mRobot, "FixedBody", 0.7);
    mContactList.clear();
    mCentroidTask = new Task(mRobot, TaskType::CENTROID);
    mCentroidTask->setGain(mCentroidTaskKp, mCentroidTaskKd);
    mJointTask = new Task(mRobot, TaskType::JOINT);
    mJointTask->setGain(mJointTaskKp, mJointTaskKd);
    mCoMRPYTask = new Task(mRobot, TaskType::COMRPY, "torso");
    mCoMRPYTask->setGain(mCoMRPYTaskKp, mCoMRPYTaskKd);
    mTaskList.clear();

    printf("[Stepping Test] Constructed\n");
}

SteppingTest::~SteppingTest() {
    delete mWBLC;
    delete mWBLCExtraData;
    delete mCentroidTask;
    delete mJointTask;
    delete mCoMRPYTask;
    delete mRfContact;
    delete mLfContact;
    delete mFixedBodyContact;
}

void SteppingTest::getTorqueInput(void * commandData_) {
    DracoCommand* cmd = (DracoCommand*) commandData_;
    cmd->q = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->qdot = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->jtrq = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());

    if (mPhaseEndTime <= mRobot->getTime()) _updatePhase();
    if (mDoUpdateContactListAndTaskList) _updateContactListandTaskList();
    _updateContact();
    _updateTask();
    _WBLCpreProcess();
    mWBLC->MakeTorque(mTaskList, mContactList, cmd->jtrq, mWBLCExtraData);
    _WBLCpostProcess();
}

void SteppingTest::_updatePhase() {
    // Update phase
    mCurrentPhase++;
    mDoUpdateContactListAndTaskList = true;

    // Update phase dependent variables
    mPhaseStartTime = mPhaseEndTime;
    mPhaseEndTime += mPhaseTimeHorizon[mCurrentPhase];
    mInitJointTask = mRobot->getQ();
    mInitCentroidTask.tail(3) = mRobot->getCoMPosition();
    std::cout << "[Phase : " << mCurrentPhase << "]" << std::endl;
    if (mCurrentPhase == mNumPhase) {
        std::cout << "Phase Over in Stepping Test" << std::endl;
        exit(0);
    }

}

void SteppingTest::_updateContactListandTaskList() {

    mTaskList.clear(); mContactList.clear();

    YAML::Node test_cfg =
        YAML::LoadFile(THIS_COM"Config/Draco/TEST/STEPPING_TEST.yaml");
    YAML::Node phase_cfg = test_cfg["phase_configuration"]["phase"+std::to_string(mCurrentPhase)];
    int num_task = myUtils::readParameter<int>(phase_cfg, "num_task");
    for (int task_id = 0; task_id < num_task; ++task_id) {
        YAML::Node task_cfg = phase_cfg["task"+std::to_string(task_id)];
        std::string task_type = myUtils::readParameter<std::string>(task_cfg, "type");
        if (task_type == "joint") {
            myUtils::readParameter(task_cfg, "target", mTargetJointTask);
            mTaskList.push_back(mJointTask);
            std::cout << "[Joint Task is Inserted]" << std::endl;
        } else if (task_type == "centroid") {
            myUtils::readParameter(task_cfg, "target", mTargetCentroidTask);
            mTargetCentroidTask.tail(3) += mRobot->getCoMPosition();
            mTaskList.push_back(mCentroidTask);
            std::cout << "[Centroid Task is Inserted]" << std::endl;
        } else if (task_type == "com_rpy") {
            myUtils::readParameter(task_cfg, "target", mTargetCoMRPYTask);
            mTargetCoMRPYTask.head(3) += mRobot->getCoMPosition();
            mTaskList.push_back(mCoMRPYTask);
            std::cout << "[CoM RPY Task is Inserted]" << std::endl;
        } else {
            std::cout << "wrong task type in stepping test" << std::endl;
            exit(0);
        }
    }

    // update contact list
    if (mCurrentPhase == 0) {
        mContactList.push_back(mFixedBodyContact);
        std::cout << "[Fixed Body Contact is Inserted]" << std::endl;
    } else {
        mContactList.push_back(mRfContact);
        std::cout << "[Right Foot Contact is Inserted]" << std::endl;
        mContactList.push_back(mLfContact);
        std::cout << "[Left Foot Contact is Inserted]" << std::endl;
    }

    mDoUpdateContactListAndTaskList = false;

}


void SteppingTest::_updateContact() {
    for (int i = 0; i < mContactList.size(); ++i)
        mContactList[i]->updateWBLCContactSpec();
}

void SteppingTest::_updateTask() {

    double t(mRobot->getTime());

    for (int i = 0; i < mTaskList.size(); ++i) {
        switch (mTaskList[i]->getType()) {
            case TaskType::JOINT:{
                                     Eigen::VectorXd joint_pos_des = Eigen::VectorXd::Zero(mTaskList[i]->getDims());
                                     Eigen::VectorXd joint_vel_des = Eigen::VectorXd::Zero(mTaskList[i]->getDims());
                                     Eigen::VectorXd joint_acc_des = Eigen::VectorXd::Zero(mTaskList[i]->getDims());
                                     if (t < mPhaseEndTime - mWaitTimeBtwPhase) {
                                         for (int i = 0; i < 10; ++i) {
                                             joint_pos_des[i] = myUtils::smooth_changing(mInitJointTask[i+6],
                                                     mTargetJointTask[i], mPhaseTimeHorizon[mCurrentPhase] - mWaitTimeBtwPhase, t - mPhaseStartTime);
                                             joint_vel_des[i] = myUtils::smooth_changing_vel(mInitJointTask[i+6],
                                                     mTargetJointTask[i], mPhaseTimeHorizon[mCurrentPhase] - mWaitTimeBtwPhase, t - mPhaseStartTime);
                                             joint_acc_des[i] = myUtils::smooth_changing_acc(mInitJointTask[i+6],
                                                     mTargetJointTask[i], mPhaseTimeHorizon[mCurrentPhase] - mWaitTimeBtwPhase, t - mPhaseStartTime);
                                         }
                                     } else {
                                         joint_pos_des = mTargetJointTask.tail(mTaskList[i]->getDims());
                                         joint_vel_des.setZero();
                                         joint_acc_des.setZero();
                                     }
                                     mTaskList[i]->updateTaskSpec(joint_pos_des,
                                             joint_vel_des, joint_acc_des);
                                     break;
                                 }
            case TaskType::CENTROID:{
                                        Eigen::VectorXd centroid_pos_des = Eigen::VectorXd::Zero(mTaskList[i]->getDims());
                                        Eigen::VectorXd centroid_vel_des = Eigen::VectorXd::Zero(mTaskList[i]->getDims());
                                        Eigen::VectorXd centroid_acc_des = Eigen::VectorXd::Zero(mTaskList[i]->getDims());
                                        if (t < mPhaseEndTime - mWaitTimeBtwPhase) {
                                            for (int i = 0; i < 3; ++i) {
                                                centroid_pos_des[i+3] = myUtils::smooth_changing(mInitCentroidTask[i+3],
                                                        mTargetCentroidTask[i+3], mPhaseTimeHorizon[mCurrentPhase] - mWaitTimeBtwPhase, t - mPhaseStartTime);
                                                centroid_vel_des[i+3] = myUtils::smooth_changing_vel(mInitCentroidTask[i+3],
                                                        mTargetCentroidTask[i+3], mPhaseTimeHorizon[mCurrentPhase] - mWaitTimeBtwPhase, t - mPhaseStartTime);
                                                centroid_acc_des[i+3] = myUtils::smooth_changing_acc(mInitCentroidTask[i+3],
                                                        mTargetCentroidTask[i+3], mPhaseTimeHorizon[mCurrentPhase] - mWaitTimeBtwPhase, t - mPhaseStartTime);
                                            }
                                        } else {
                                            centroid_pos_des = mTargetCentroidTask;
                                            centroid_vel_des.setZero();
                                            centroid_acc_des.setZero();
                                        }
                                         mTaskList[i]->updateTaskSpec(centroid_pos_des,
                                                 mRobot->getRobotMass() * centroid_vel_des,
                                                 mRobot->getRobotMass() * centroid_acc_des);
                                         break;
                                    }
            case TaskType::COMRPY:{
                                      Eigen::VectorXd com_rpy_pos_des = Eigen::VectorXd::Zero(7);
                                      Eigen::VectorXd com_rpy_vel_des = Eigen::VectorXd::Zero(6);
                                      Eigen::VectorXd com_rpy_acc_des = Eigen::VectorXd::Zero(6);

                                      com_rpy_pos_des = mTargetCoMRPYTask;

                                      mTaskList[i]->updateTaskSpec(com_rpy_pos_des,
                                                                   com_rpy_vel_des,
                                                                   com_rpy_acc_des);
                                  }
            default:
                break;
        }
    }
}

void SteppingTest::_WBLCpreProcess() {
    // dynamic property setting
    mWBLC->UpdateSetting(mRobot->getMassMatrix(),
                         mRobot->getInvMassMatrix(),
                         mRobot->getCoriolis(),
                         mRobot->getGravity());
    // cost setting
    int taskDim(0);
    int contactDim(0);
    taskDim += mTaskList[0]->getDims();
    for (int i = 0; i < mContactList.size(); ++i)
        contactDim += 6;
    mWBLCExtraData->cost_weight = Eigen::VectorXd::Zero(taskDim + contactDim);

    for(int i(0); i<taskDim; ++i) {
        mWBLCExtraData->cost_weight[i] = 1000000.0;
    }
    for(int i(0); i<contactDim; ++i){
        mWBLCExtraData->cost_weight[taskDim + i] = 1.0;
        if (i % 6 == 5) {
            mWBLCExtraData->cost_weight[taskDim + i] = 0.001; // vertical reaction
        }
    }
}

void SteppingTest::_WBLCpostProcess() {
    for(int i(0); i<mTaskList.size(); ++i)
        mTaskList[i]->unsetTaskSpec();
    for(int i(0); i<mContactList.size(); ++i)
        mContactList[i]->unsetWBLCContactSpec();
}

void SteppingTest::initialize() {
    // Planner Initialize
    mTestInitQ = mRobot->getQ();
    mTestInitCoMPos = mRobot->getCoMPosition();
    mTestInitTime = mRobot->getTime();

    // Controller Initialize

    // Initialize phase dependent variables
    mInitJointTask = mRobot->getQ();
    mInitCentroidTask = Eigen::VectorXd::Zero(6);
    mInitCentroidTask.tail(3) = mRobot->getCoMPosition();
    mPhaseStartTime = mTestInitTime;
    mPhaseEndTime = mTestInitTime + mPhaseTimeHorizon[0];

    isInitialized = true;
}
