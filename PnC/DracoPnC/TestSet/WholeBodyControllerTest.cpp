#include "DracoPnC/TestSet/TestSet.hpp"
#include "RobotSystem.hpp"
#include "Task.hpp"
#include "WBLC.hpp"
#include "WBLCContact.hpp"
#include "Utilities.hpp"

WholeBodyControllerTest::WholeBodyControllerTest(RobotSystem* robot_): Test(robot_) {
    // Choose Planner

    // Choose Controller
    std::vector<bool> act_list;
    act_list.resize(mRobot->getNumDofs(), true);
    for(int i(0); i < mRobot->getNumVirtualDofs(); ++i)
        act_list[i] = false;

    mWBLC= new WBLC(act_list);
    mWBLCExtraData= new WBLC_ExtraData();


    printf("[Whole Body Controller Test] Constructed\n");
}

WholeBodyControllerTest::~WholeBodyControllerTest() {
    delete mWBLC;
    delete mWBLCExtraData;
    delete mCoMTask;
    delete mRfContact;
    delete mLfContact;
}

void WholeBodyControllerTest::getTorqueInput(void * commandData_) {
    DracoCommand* cmd = (DracoCommand*) commandData_;
    cmd->q = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->qdot = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->jtrq = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());

    _updateContact();
    _updateTask();
    _WBLCpreProcess();
    mWBLC->MakeTorque(mTaskList, mContactList, cmd->jtrq, mWBLCExtraData);
    _WBLCpostProcess();

}

void WholeBodyControllerTest::initialize() {
    //Planner Initialize
    mMid = mRobot->getCoMPosition();
    mAmp.setZero(); mFreq.setZero();
    mMid[2] -= 0.08;
    mAmp[2] = 0.02; mFreq[2] = 0.5;
    mInterpolationDuration = 1.0;
    mTestInitTime = (mRobot->getTime());
    Eigen::VectorXd com_pos = mRobot->getCoMPosition();
    Eigen::VectorXd com_vel = mRobot->getCoMVelocity();
    double ini[9] = {com_pos[0], com_pos[1], com_pos[2],
                     com_vel[0], com_vel[1], com_vel[2],
                     0., 0., 0.};
    double fin[9] = {mMid[0], mMid[1], mMid[2],
                     mAmp[0]*2*M_PI*mFreq[0], mAmp[1]*2*M_PI*mFreq[1], mAmp[2]*2*M_PI*mFreq[2],
                     0., 0., 0.};
    double **middle_pt;
    mSpline.SetParam(ini, fin, middle_pt, mInterpolationDuration);

    //Controller Initialize

    mCoMTask = new Task(mRobot, TaskType::CENTROID);
    //mJointTask = new Task(mRobot, TaskType::JOINT);
    mTaskList.clear();
    mTaskList.push_back(mCoMTask);
    //mTaskList.push_back(mJointTask);

    mRfContact = new WBLCContact(mRobot, "rAnkle", 0.7);
    mLfContact = new WBLCContact(mRobot, "lAnkle", 0.7);
    mContactList.clear();
    mContactList.push_back(mRfContact);
    mContactList.push_back(mLfContact);

    isInitialized = true;
}

void WholeBodyControllerTest::_updateContact() {
    for (int i = 0; i < mContactList.size(); ++i)
        mContactList[i]->updateWBLCContactSpec();
}

//Joint Position Task
/*
void WholeBodyControllerTest::_updateTask() {
    Eigen::VectorXd pos = (mRobot->getInitialConfiguration()).tail(mRobot->getNumActuatedDofs());
    Eigen::VectorXd vel = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
    Eigen::VectorXd acc = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
    mTaskList[0]->updateTaskSpec(pos, vel, acc);
}
*/


// Centroid Task

void WholeBodyControllerTest::_updateTask() {
    Eigen::VectorXd pos = Eigen::VectorXd::Zero(mTaskList[0]->getDims());
    Eigen::VectorXd vel = Eigen::VectorXd::Zero(mTaskList[0]->getDims());
    Eigen::VectorXd acc = Eigen::VectorXd::Zero(mTaskList[0]->getDims());
    double t = mRobot->getTime();
    static double d_ary[3];
    if(t < mTestInitTime + mInterpolationDuration) {
        mSpline.getCurvePoint(t - mTestInitTime, d_ary);
        for (int i = 0; i < 3; ++i) pos[i+3] = d_ary[i];
        mSpline.getCurveDerPoint(t - mTestInitTime, 1, d_ary);
        for (int i = 0; i < 3; ++i) vel[i+3] = d_ary[i];
        mSpline.getCurveDerPoint(t - mTestInitTime, 2, d_ary);
        for (int i = 0; i < 3; ++i) acc[i+3] = d_ary[i];
    } else {
        Eigen::VectorXd p, v, a;
        myUtils::getSinusoidTrajectory(mTestInitTime + mInterpolationDuration,
                                       mMid, mAmp, mFreq, t, p, v, a);
        pos.tail(3) = p; vel.tail(3) = v; acc.tail(3) = a;
    }
    mTaskList[0]->updateTaskSpec(pos, vel, acc);
}

// Multi Task
/*
void WholeBodyControllerTest::_updateTask() {
    Eigen::VectorXd pos = Eigen::VectorXd::Zero(mTaskList[0]->getDims());
    Eigen::VectorXd vel = Eigen::VectorXd::Zero(mTaskList[0]->getDims());
    Eigen::VectorXd acc = Eigen::VectorXd::Zero(mTaskList[0]->getDims());
    //double t = mRobot->getTime();
    //static double d_ary[3];
    //if(t < mTestInitTime + mInterpolationDuration) {
        //mSpline.getCurvePoint(t - mTestInitTime, d_ary);
        //for (int i = 0; i < 3; ++i) pos[i+3] = d_ary[i];
        //mSpline.getCurveDerPoint(t - mTestInitTime, 1, d_ary);
        //for (int i = 0; i < 3; ++i) vel[i+3] = d_ary[i];
        //mSpline.getCurveDerPoint(t - mTestInitTime, 2, d_ary);
        //for (int i = 0; i < 3; ++i) acc[i+3] = d_ary[i];
    //} else {
        //Eigen::VectorXd p, v, a;
        //myUtils::getSinusoidTrajectory(mTestInitTime + mInterpolationDuration,
                                       //mMid, mAmp, mFreq, t, p, v, a);
        //pos.tail(3) = p; vel.tail(3) = v; acc.tail(3) = a;
    //}
    //mTaskList[0]->updateTaskSpec(pos, vel, acc);

    //TEST
    static Eigen::VectorXd com_pos_des = mRobot->getCoMPosition();
    pos.tail(3) = com_pos_des;
    mTaskList[0]->updateTaskSpec(pos, vel, acc);
    //TEST END

    Eigen::VectorXd pos_ = (mRobot->getInitialConfiguration()).tail(mRobot->getNumActuatedDofs());
    Eigen::VectorXd vel_ = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
    Eigen::VectorXd acc_ = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
    mTaskList[1]->updateTaskSpec(pos_, vel_, acc_);
}
*/

void WholeBodyControllerTest::_WBLCpreProcess() {
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

void WholeBodyControllerTest::_WBLCpostProcess() {
    for(int i(0); i<mTaskList.size(); ++i)
        mTaskList[i]->unsetTaskSpec();
    for(int i(0); i<mContactList.size(); ++i)
        mContactList[i]->unsetWBLCContactSpec();
}