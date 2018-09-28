#include "PnC/DracoPnC/TestSet/TestSet.hpp"
#include "RobotSystem/RobotSystem.hpp"
#include "PnC/WBC/Task.hpp"
#include "PnC/WBC/WBLC/WBLC.hpp"
#include "PnC/WBC/WBLC/WBLCContact.hpp"
#include "Utils/Utilities.hpp"

AdmittanceTest::AdmittanceTest(RobotSystem* robot_) : Test(robot_) {

    // Choose Controller
    std::vector<bool> act_list;
    act_list.resize(mRobot->getNumDofs(), true);
    for(int i(0); i < mRobot->getNumVirtualDofs(); ++i)
        act_list[i] = false;
    mWBLC = new WBLC(act_list);
    mWBLCExtraData= new WBLC_ExtraData();
    mRfContact = new WBLCContact(mRobot, "rAnkle", 0.7);
    mLfContact = new WBLCContact(mRobot, "lAnkle", 0.7);
    mContactList.clear();
    mContactList.push_back(mRfContact);
    mContactList.push_back(mLfContact);
    mCentroidTask = new Task(mRobot, TaskType::CENTROID);
    mJointTask = new Task(mRobot, TaskType::JOINT);
    mTaskList.clear();
    mTaskList.push_back(mCentroidTask);
    mTaskList.push_back(mJointTask);

    printf("[Admittance Test] Constructed\n");
}

AdmittanceTest::~AdmittanceTest() {
    delete mWBLC;
    delete mWBLCExtraData;
    delete mCentroidTask;
    delete mJointTask;
    delete mRfContact;
    delete mLfContact;
}

void AdmittanceTest::getTorqueInput(void * commandData_) {
    DracoCommand* cmd = (DracoCommand*) commandData_;
    cmd->q = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->qdot = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->jtrq = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());

    _updateContact();
    _updateTask();
    _WBLCpreProcess();
    mWBLC->MakeTorque(mTaskList, mContactList, cmd->jtrq, mWBLCExtraData);
    _WBLCpostProcess();

    // Admittance
    // Option 1
    //Eigen::VectorXd qddot_des = mWBLC->getQddot();
    //Eigen::VectorXd qdot_des = myUtils::eulerIntegration(mRobot->getQdot(), qddot_des, SERVO_RATE);
    //Eigen::VectorXd q_des = myUtils::eulerIntegration(mRobot->getQ(), qdot_des, SERVO_RATE);
    //cmd->q = q_des;
    //cmd->qdot = qdot_des;
    // Option 2 : seems different from option 1 due to delta
    cmd->qdot = mWBLC->getQdot();
    cmd->q = myUtils::doubleIntegration(mRobot->getQ(), mWBLC->getQdot(), mWBLC->getQddot(), SERVO_RATE);

}

void AdmittanceTest::_updateContact() {
    for (int i = 0; i < mContactList.size(); ++i)
        mContactList[i]->updateWBLCContactSpec();
}

void AdmittanceTest::_updateTask() {

    double t(mRobot->getTime());

    ///////////////////
    // 1. Centroid Task
    ///////////////////

    Eigen::VectorXd centroid_pos_des = Eigen::VectorXd::Zero(mTaskList[0]->getDims());
    Eigen::VectorXd centroid_vel_des = Eigen::VectorXd::Zero(mTaskList[0]->getDims());
    Eigen::VectorXd centroid_acc_des = Eigen::VectorXd::Zero(mTaskList[0]->getDims());
    if (t < mTestInitTime + mInterpolationDuration) {
        for (int i = 0; i < 3; ++i) {
            centroid_pos_des[i+3] = myUtils::smooth_changing(mTestInitCoMPos[i],
                    mNominalCentroidState[i+3], mInterpolationDuration, t);
            centroid_vel_des[i+3] = myUtils::smooth_changing_vel(mTestInitCoMPos[i],
                    mNominalCentroidState[i+3], mInterpolationDuration, t);
            centroid_acc_des[i+3] = myUtils::smooth_changing_acc(mTestInitCoMPos[i],
                    mNominalCentroidState[i+3], mInterpolationDuration, t);
        }
    } else {
        myUtils::getSinusoidTrajectory(mTestInitTime + mInterpolationDuration,
                mMid, mAmp, mFreq, t, centroid_pos_des, centroid_vel_des,
                centroid_acc_des);
    }
    mTaskList[0]->updateTaskSpec(centroid_pos_des,
                                 mRobot->getRobotMass() * centroid_vel_des,
                                 mRobot->getRobotMass() * centroid_acc_des);

    ////////////////
    // 2. Joint Task
    ////////////////

    Eigen::VectorXd joint_pos_des = Eigen::VectorXd::Zero(mTaskList[1]->getDims());
    Eigen::VectorXd joint_vel_des = Eigen::VectorXd::Zero(mTaskList[1]->getDims());
    Eigen::VectorXd joint_acc_des = Eigen::VectorXd::Zero(mTaskList[1]->getDims());
    if(t < mTestInitTime + mInterpolationDuration) {
        for (int i = 0; i < mTaskList[1]->getDims(); ++i) {
            joint_pos_des[i] = myUtils::smooth_changing(mTestInitQ[i],
                    mNominalJointState[i], mInterpolationDuration, t);
            joint_vel_des[i] = myUtils::smooth_changing_vel(mTestInitQ[i],
                    mNominalJointState[i], mInterpolationDuration, t);
            joint_acc_des[i] = myUtils::smooth_changing_acc(mTestInitQ[i],
                    mNominalJointState[i], mInterpolationDuration, t);
        }
    } else {
        joint_pos_des = mNominalJointState;
        joint_vel_des.setZero();
        joint_acc_des.setZero();
    }
    mTaskList[1]->updateTaskSpec(joint_pos_des, joint_vel_des, joint_acc_des); // Any way no left dimension
    Eigen::VectorXd vec_zero = Eigen::VectorXd::Zero(10);
}

void AdmittanceTest::_WBLCpreProcess() {
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
    }
    mWBLCExtraData->cost_weight[taskDim + 5] = 0.001; // vertical reaction
    mWBLCExtraData->cost_weight[taskDim + 11] = 0.001; // vertical reaction
}

void AdmittanceTest::_WBLCpostProcess() {
    for(int i(0); i<mTaskList.size(); ++i)
        mTaskList[i]->unsetTaskSpec();
    for(int i(0); i<mContactList.size(); ++i)
        mContactList[i]->unsetWBLCContactSpec();
}

void AdmittanceTest::initialize() {
    //Planner Initialize
    mTestInitQ = mRobot->getQ();
    mTestInitCoMPos = mRobot->getCoMPosition();
    mTestInitTime = mRobot->getTime();

    try {
        YAML::Node test_cfg =
            YAML::LoadFile(THIS_COM"Config/Draco/TEST/ADMITTANCE_TEST.yaml");
        YAML::Node control_cfg = test_cfg["control_configuration"];
        myUtils::readParameter(control_cfg, "centroid_task_kp", mCentroidTaskKp);
        myUtils::readParameter(control_cfg, "centroid_task_kd", mCentroidTaskKd);
        myUtils::readParameter(control_cfg, "joint_task_kp", mJointTaskKp);
        myUtils::readParameter(control_cfg, "joint_task_kd", mJointTaskKd);
        YAML::Node planner_cfg = test_cfg["planner_configuration"];
        myUtils::readParameter(planner_cfg, "transition_time", mInterpolationDuration);
        myUtils::readParameter(planner_cfg, "sinusoid_mid", mNominalCentroidState);
        myUtils::readParameter(planner_cfg, "nominal_joint_state", mNominalJointState);
        myUtils::readParameter(planner_cfg, "sinusoid_amp", mAmp);
        myUtils::readParameter(planner_cfg, "sinusoid_mid", mMid);
        myUtils::readParameter(planner_cfg, "sinusoid_freq", mFreq);

    }catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }

    isInitialized = true;
}
