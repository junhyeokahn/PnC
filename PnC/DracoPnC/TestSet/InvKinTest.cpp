#include "DracoPnC/TestSet/TestSet.hpp"
#include "RobotSystem.hpp"
#include "Utilities.hpp"
#include "WBLC.hpp"
#include "WBLCContact.hpp"

InvKinTest::InvKinTest(RobotSystem* robot_): Test(robot_) {
    // Choose Planner
    mDrakeModel = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                THIS_COM"RobotSystem/RobotModel/Robot/Draco/DracoNoVisualDrake.urdf",
                drake::multibody::joints::kFixed, mDrakeModel.get());
    mRfIdx = mDrakeModel->FindBodyIndex("rAnkle");
    mLfIdx = mDrakeModel->FindBodyIndex("lAnkle");
    mWorldIdx = 0;

    // Choose Controller
    std::vector<bool> act_list;
    act_list.resize(mRobot->getNumDofs(), true);
    for(int i(0); i < mRobot->getNumVirtualDofs(); ++i)
        act_list[i] = false;

    mWBLC = new WBLC(act_list);
    mWBLCExtraData= new WBLC_ExtraData();
    mJointTask = new Task(mRobot, TaskType::JOINT);
    mTaskList.clear();
    mTaskList.push_back(mJointTask);

    printf("[Inv Kin Test] Constructed\n");
}

InvKinTest::~InvKinTest() {
    delete mWBLC;
    delete mWBLCExtraData;
    delete mJointTask;
    delete mRfContact;
    delete mLfContact;
}

void InvKinTest::getTorqueInput(void * commandData_) {
    DracoCommand* cmd = (DracoCommand*) commandData_;
    cmd->q = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->qdot = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->jtrq = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());

    //////////////
    // constraints
    //////////////
    Eigen::Vector2d tspan(0, 1);
    std::vector<const RigidBodyConstraint*> constraint_array;

    // 1. Foot position constraint
    double posTol(0.00001);
    Eigen::Vector3d rFootPosLb = mInitRfIso.translation() - Eigen::Vector3d::Constant(posTol);
    Eigen::Vector3d rFootPosUb = mInitRfIso.translation() + Eigen::Vector3d::Constant(posTol);
    Eigen::Vector3d lFootPosLb = mInitLfIso.translation() - Eigen::Vector3d::Constant(posTol);
    Eigen::Vector3d lFootPosUb = mInitLfIso.translation() + Eigen::Vector3d::Constant(posTol);
    WorldPositionConstraint rFwpc(mDrakeModel.get(), mRfIdx,
            Eigen::Vector3d::Zero(), rFootPosLb, rFootPosUb, tspan);
    constraint_array.push_back(&rFwpc);
    WorldPositionConstraint lFwpc(mDrakeModel.get(), mLfIdx,
            Eigen::Vector3d::Zero(), lFootPosLb, lFootPosUb, tspan);
    constraint_array.push_back(&lFwpc);

    // 2. CoM constraint
    static double d_ary[3];
    Eigen::Vector3d comDes;
    double t(mRobot->getTime());
    if(t < mTestInitTime + mInterpolationDuration) {
        mSpline.getCurvePoint(t - mTestInitTime, d_ary);
        for (int i = 0; i < 3; ++i) comDes[i] = d_ary[i];
    } else {
        Eigen::VectorXd p, v, a;
        myUtils::getSinusoidTrajectory(mTestInitTime + mInterpolationDuration,
                mMid, mAmp, mFreq, t, p, v, a);
        comDes = p;
    }
    Eigen::Vector3d comLb = comDes - Eigen::Vector3d::Constant(posTol);
    Eigen::Vector3d comUb = comDes + Eigen::Vector3d::Constant(posTol);

    WorldCoMConstraint CoMc(mDrakeModel.get(), comLb, comUb, tspan);
    constraint_array.push_back(&CoMc);

    const IKoptions ikoptions(mDrakeModel.get());
    Eigen::VectorXd qSol = Eigen::VectorXd::Zero(mDrakeModel->get_num_positions());
    int info = 0;
    std::vector<std::string> infeasible_constraint;
    //inverseKin(mDrakeModel.get(), mInitQ, mInitQ, constraint_array.size(),
    inverseKin(mDrakeModel.get(), mPrevSol, mInitQ, constraint_array.size(),
            constraint_array.data(), ikoptions, &qSol, &info,
            &infeasible_constraint);
    if (info == 1) {
        cmd->q = qSol;
        //cmd->qdot.setZero();
        cmd->qdot = (qSol - mPrevSol) / 1500.0;
        cmd->jtrq.setZero();
        mPrevSol = qSol;

    } else {
        std::cout << "[Inverse Kinematic Solver Falied] INFO : " << info << std::endl;
        exit(0);
    }

    std::cout << "des com" << std::endl;
    std::cout << comDes << std::endl;
    std::cout << "qsol com" << std::endl;
    KinematicsCache<double> cache = mDrakeModel->doKinematics(qSol);
    std::cout << mDrakeModel->centerOfMass(cache) << std::endl;
    std::cout << "act com" << std::endl;
    std::cout << mRobot->getCoMPosition() << std::endl;
    //std::cout << "qSol" << std::endl;
    //std::cout << qSol << std::endl;
    //std::cout << "dart q" << std::endl;
    //std::cout << mRobot->getQ() << std::endl;

    //update contact
    _updateContact();
    // update task
    Eigen::VectorXd qdotSol = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
    Eigen::VectorXd qddotSol = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
    mTaskList[0]->updateTaskSpec(qSol.tail(mRobot->getNumActuatedDofs()), qdotSol, qddotSol);
    //mTaskList[0]->updateTaskSpec(mInitQ.tail(mRobot->getNumActuatedDofs()), qdotSol, qddotSol);
    _WBLCpreProcess();
    mWBLC->MakeTorque(mTaskList, mContactList, cmd->jtrq, mWBLCExtraData);
    _WBLCpostProcess();
}

void InvKinTest::_updateContact() {
    for (int i = 0; i < mContactList.size(); ++i)
        mContactList[i]->updateWBLCContactSpec();
}


void InvKinTest::_WBLCpostProcess() {
    for(int i(0); i<mTaskList.size(); ++i)
        mTaskList[i]->unsetTaskSpec();
    for(int i(0); i<mContactList.size(); ++i)
        mContactList[i]->unsetWBLCContactSpec();
}

void InvKinTest::_WBLCpreProcess() {
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


void InvKinTest::initialize() {
    //Planner Initialize
    mInitQ = mRobot->getInitialConfiguration();
    mPrevSol = mInitQ;
    KinematicsCache<double> cache = mDrakeModel->doKinematics(mInitQ);
    mInitRfIso = mDrakeModel->relativeTransform(cache, mWorldIdx, mRfIdx);
    mInitLfIso = mDrakeModel->relativeTransform(cache, mWorldIdx, mLfIdx);
    mInitCOM = mDrakeModel->centerOfMass(cache);

    //Controller Initialize
    mMid = mInitCOM;
    mAmp.setZero();
    mFreq.setZero();
    //mMid[0] += 0.05;
    mMid[2] -= 0.01;
    mMid[0] = (mInitRfIso.translation())[0];
    //mAmp[2] = 0.03;
    mAmp[2] = 0.0;
    mFreq[2] = 0.06;
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

    mRfContact = new WBLCContact(mRobot, "rAnkle", 0.7); // TODO : construct it at the beginning and initialize it here
    mLfContact = new WBLCContact(mRobot, "lAnkle", 0.7);
    mContactList.clear();
    mContactList.push_back(mRfContact);
    mContactList.push_back(mLfContact);

    isInitialized = true;
}
