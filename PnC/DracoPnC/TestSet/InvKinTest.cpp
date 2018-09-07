#include "DracoPnC/TestSet/TestSet.hpp"
#include "RobotSystem.hpp"
#include "Utilities.hpp"
#include "WBLC.hpp"
#include "WBLCContact.hpp"
#include "pseudo_inverse.hpp"
#include "DataManager.hpp"

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

    mCoMPosDes = Eigen::VectorXd::Zero(3);
    mCoMVelDes = Eigen::VectorXd::Zero(3);
    mCoMPosAct = Eigen::VectorXd::Zero(3);
    mCoMVelAct = Eigen::VectorXd::Zero(3);
    mCoMPosSol = Eigen::VectorXd::Zero(3);
    mQSol = Eigen::VectorXd::Zero(16);
    mQAct = Eigen::VectorXd::Zero(16);
    mQdotSol = Eigen::VectorXd::Zero(16);
    mQdotAct = Eigen::VectorXd::Zero(16);

    DataManager* dataManager = DataManager::GetDataManager();
    dataManager->RegisterData(&mCoMPosDes, VECT, "CoMPosDes", 3);
    dataManager->RegisterData(&mCoMVelDes, VECT, "CoMVelDes", 3);
    dataManager->RegisterData(&mCoMPosAct, VECT, "CoMPosAct", 3);
    dataManager->RegisterData(&mCoMVelAct, VECT, "CoMVelAct", 3);
    dataManager->RegisterData(&mCoMPosSol, VECT, "CoMPosSol", 3);
    dataManager->RegisterData(&mQSol, VECT, "QSol", 16);
    dataManager->RegisterData(&mQAct, VECT, "QAct", 16);
    dataManager->RegisterData(&mQdotSol, VECT, "QdotSol", 16);
    dataManager->RegisterData(&mQdotAct, VECT, "QdotAct", 16);

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

    // 1. Foot position & orientation constraint
    double posTol(0.00001);
    Eigen::Isometry3d rFIso = mRobot->getBodyNodeIsometry("rAnkle");
    Eigen::Isometry3d lFIso = mRobot->getBodyNodeIsometry("lAnkle");
    Eigen::Vector3d rFootPosLb = rFIso.translation() - Eigen::Vector3d::Constant(posTol);
    Eigen::Vector3d rFootPosUb = rFIso.translation() + Eigen::Vector3d::Constant(posTol);
    Eigen::Vector3d lFootPosLb = lFIso.translation() - Eigen::Vector3d::Constant(posTol);
    Eigen::Vector3d lFootPosUb = lFIso.translation() + Eigen::Vector3d::Constant(posTol);
    WorldPositionConstraint rFwpc(mDrakeModel.get(), mRfIdx,
            Eigen::Vector3d::Zero(), rFootPosLb, rFootPosUb, tspan);
    constraint_array.push_back(&rFwpc);
    WorldPositionConstraint lFwpc(mDrakeModel.get(), mLfIdx,
            Eigen::Vector3d::Zero(), lFootPosLb, lFootPosUb, tspan);
    constraint_array.push_back(&lFwpc);

    Eigen::Quaternion<double> rFQuat(rFIso.linear());
    Eigen::Quaternion<double> lFQuat(lFIso.linear());
    //Eigen::Vector4d rFQuatDes(rFQuat.w(), rFQuat.x(), rFQuat.y(), rFQuat.z());
    //Eigen::Vector4d lFQuatDes(lFQuat.w(), lFQuat.x(), lFQuat.y(), lFQuat.z());
    Eigen::Vector4d rFQuatDes(1, 0, 0, 0);
    Eigen::Vector4d lFQuatDes(1, 0, 0, 0);
    double quatTol = 0.0017453292519943296;
    WorldQuatConstraint rFwqc(mDrakeModel.get(), mRfIdx, rFQuatDes, quatTol,
            tspan);
    constraint_array.push_back(&rFwqc);
    WorldQuatConstraint lFwqc(mDrakeModel.get(), mLfIdx, lFQuatDes, quatTol,
            tspan);
    constraint_array.push_back(&rFwqc);

    // 2. CoM constraint
    static double d_ary[3];
    Eigen::Vector3d comPosDes;
    Eigen::Vector3d comVelDes;
    double t(mRobot->getTime());
    if(t < mTestInitTime + mInterpolationDuration) {
        mSpline.getCurvePoint(t - mTestInitTime, d_ary);
        for (int i = 0; i < 3; ++i) comPosDes[i] = d_ary[i];
        mSpline.getCurveDerPoint(t - mTestInitTime, 1, d_ary);
        for (int i = 0; i < 3; ++i) comVelDes[i] = d_ary[i];
    } else {
        Eigen::VectorXd p, v, a;
        myUtils::getSinusoidTrajectory(mTestInitTime + mInterpolationDuration,
                mMid, mAmp, mFreq, t, p, v, a);
        comPosDes = p;
        comVelDes = v;
    }

    // TODO : TEST
    comVelDes.setZero();
    // TODO : TEST END
    Eigen::Vector3d comLb = comPosDes - Eigen::Vector3d::Constant(posTol);
    Eigen::Vector3d comUb = comPosDes + Eigen::Vector3d::Constant(posTol);

    WorldCoMConstraint CoMc(mDrakeModel.get(), comLb, comUb, tspan);
    constraint_array.push_back(&CoMc);

    // Ik option
    IKoptions ikoptions(mDrakeModel.get());
    Eigen::MatrixXd Q =
        Eigen::MatrixXd::Identity(mDrakeModel->get_num_positions(), mDrakeModel->get_num_positions());
    //for (int i = 0; i < 6; ++i) {
        //Q(i, i) = 100;
    //}
    //ikoptions.setQ(Q);
    //ikoptions.setMajorIterationsLimit(1500);
    //ikoptions.setIterationsLimit(20000);

    Eigen::VectorXd qSol = Eigen::VectorXd::Zero(mDrakeModel->get_num_positions());
    int info = 0;
    std::vector<std::string> infeasible_constraint;
    //inverseKin(mDrakeModel.get(), mInitQ, mInitQ, constraint_array.size(),
    inverseKin(mDrakeModel.get(), mPrevSol, mPrevSol, constraint_array.size(),
            constraint_array.data(), ikoptions, &qSol, &info,
            &infeasible_constraint);
    if (info == 1) {
        cmd->q = qSol;
        Eigen::MatrixXd JrFlF(12, mRobot->getNumDofs());
        JrFlF.block(0, 0, 6, mRobot->getNumDofs()) =
            mRobot->getBodyNodeJacobian("rAnkle");
        JrFlF.block(6, 0, 6, mRobot->getNumDofs()) =
            mRobot->getBodyNodeJacobian("lAnkle");
        Eigen::MatrixXd JNullrFlF = myUtils::getNullSpace(JrFlF);
        Eigen::MatrixXd JcomNull = ((mRobot->getCoMJacobian()).block(3, 0, 3, mRobot->getNumDofs()))*JNullrFlF;
        Eigen::MatrixXd JcomNullInv;
        myUtils::pseudoInverse(JcomNull, 0.0001, JcomNullInv);
        cmd->qdot = JcomNullInv* comVelDes;
        mPrevSol = qSol;
        //_checkIKResult(cmd->q, cmd->qdot);

    } else {
        std::cout << "[Inverse Kinematic Solver Falied] INFO : " << info << std::endl;
        exit(0);
    }

    mCoMPosDes = comPosDes;
    mCoMVelDes = comVelDes;
    mCoMPosAct = mRobot->getCoMPosition();
    mCoMVelAct = mRobot->getCoMVelocity();
    KinematicsCache<double> cache = mDrakeModel->doKinematics(qSol);
    mCoMPosSol = mDrakeModel->centerOfMass(cache);
    mQSol = cmd->q;
    mQdotSol = cmd->qdot;
    mQAct = mRobot->getQ();
    mQdotAct = mRobot->getQdot();

    //update contact
    _updateContact();
    // update task
    Eigen::VectorXd qdotSol = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
    Eigen::VectorXd qddotSol = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());
    //mTaskList[0]->updateTaskSpec(qSol.tail(mRobot->getNumActuatedDofs()), cmd->qdot.tail(mRobot->getNumActuatedDofs()), qddotSol);
    mTaskList[0]->updateTaskSpec(mInitQ.tail(mRobot->getNumActuatedDofs()), qdotSol, qddotSol);
    _WBLCpreProcess();
    mWBLC->MakeTorque(mTaskList, mContactList, cmd->jtrq, mWBLCExtraData);
    _WBLCpostProcess();
}

void InvKinTest::_checkIKResult(const Eigen::VectorXd q, const Eigen::VectorXd qdot) {
    KinematicsCache<double> cache = mDrakeModel->doKinematics(q);
    RobotSystem dartModel = RobotSystem(6, THIS_COM"RobotSystem/RobotModel/Robot/Draco/DracoNoVisualDart.urdf");
    dartModel.updateSystem(0.0, q, qdot, true);
    std::cout << "************** IK Solution Check ****************" << std::endl;

    std::cout << "[Configuration]" << std::endl;
    std::cout << "sol" << std::endl;
    std::cout << q << std::endl;
    std::cout << "actual" << std::endl;
    std::cout << mRobot->getQ() << std::endl;
    std::cout << "error" << std::endl;
    std::cout << q - mRobot->getQ() << std::endl;

    std::cout << "[CoM]" << std::endl;
    std::cout << "desired" << std::endl;
    std::cout << mCoMPosDes << std::endl;
    std::cout << "sol" << std::endl;
    std::cout << mDrakeModel->centerOfMass(cache) << std::endl;
    std::cout << "actual robot" << std::endl;
    std::cout << mRobot->getCoMPosition() << std::endl;

    std::cout << "[Right Foot Pos]" << std::endl;
    std::cout << "desired" << std::endl;
    std::cout << dartModel.getBodyNodeIsometry("rAnkle").translation() << std::endl;
    std::cout << "sol" << std::endl;
    std::cout << mDrakeModel->relativeTransform(cache, mWorldIdx, mRfIdx).translation() << std::endl;
    std::cout << "actual robot" << std::endl;
    std::cout << mRobot->getBodyNodeIsometry("rAnkle").translation() << std::endl;

    std::cout << "[Right Foot Ori]" << std::endl;
    std::cout << "desired" << std::endl;
    std::cout << dartModel.getBodyNodeIsometry("rAnkle").linear() << std::endl;
    std::cout << "sol" << std::endl;
    std::cout << mDrakeModel->relativeTransform(cache, mWorldIdx, mRfIdx).linear() << std::endl;
    std::cout << "actual robot" << std::endl;
    std::cout << mRobot->getBodyNodeIsometry("rAnkle").linear() << std::endl;

    std::cout << "[Right Knee Pos]" << std::endl;
    std::cout << "desired" << std::endl;
    std::cout << dartModel.getBodyNodeIsometry("rKnee").translation() << std::endl;
    std::cout << "sol" << std::endl;
    std::cout << mDrakeModel->relativeTransform(cache, mWorldIdx, mDrakeModel->FindBodyIndex("rKnee")).translation() << std::endl;
    std::cout << "actual robot" << std::endl;
    std::cout << mRobot->getBodyNodeIsometry("rKnee").translation() << std::endl;

    std::cout << "[Right Knee Ori]" << std::endl;
    std::cout << "desired" << std::endl;
    std::cout << dartModel.getBodyNodeIsometry("rKnee").linear() << std::endl;
    std::cout << "sol" << std::endl;
    std::cout << mDrakeModel->relativeTransform(cache, mWorldIdx, mDrakeModel->FindBodyIndex("rKnee")).linear() << std::endl;
    std::cout << "actual robot" << std::endl;
    std::cout << mRobot->getBodyNodeIsometry("rKnee").linear() << std::endl;

    std::cout << "[Left Foot Pos]" << std::endl;
    std::cout << "desired" << std::endl;
    std::cout << dartModel.getBodyNodeIsometry("lAnkle").translation() << std::endl;
    std::cout << "sol" << std::endl;
    std::cout << mDrakeModel->relativeTransform(cache, mWorldIdx, mLfIdx).translation() << std::endl;
    std::cout << "actual robot" << std::endl;
    std::cout << mRobot->getBodyNodeIsometry("lAnkle").translation() << std::endl;

    std::cout << "[Left Foot Ori]" << std::endl;
    std::cout << "desired" << std::endl;
    std::cout << dartModel.getBodyNodeIsometry("lAnkle").linear() << std::endl;
    std::cout << "sol" << std::endl;
    std::cout << mDrakeModel->relativeTransform(cache, mWorldIdx, mLfIdx).linear() << std::endl;
    std::cout << "actual robot" << std::endl;
    std::cout << mRobot->getBodyNodeIsometry("lAnkle").linear() << std::endl;

    std::cout << "[Left Knee Pos]" << std::endl;
    std::cout << "desired" << std::endl;
    std::cout << dartModel.getBodyNodeIsometry("lKnee").translation() << std::endl;
    std::cout << "sol" << std::endl;
    std::cout << mDrakeModel->relativeTransform(cache, mWorldIdx, mDrakeModel->FindBodyIndex("rKnee")).translation() << std::endl;
    std::cout << "actual robot" << std::endl;
    std::cout << mRobot->getBodyNodeIsometry("lKnee").translation() << std::endl;

    std::cout << "[Left Knee Ori]" << std::endl;
    std::cout << "desired" << std::endl;
    std::cout << dartModel.getBodyNodeIsometry("lKnee").linear() << std::endl;
    std::cout << "sol" << std::endl;
    std::cout << mDrakeModel->relativeTransform(cache, mWorldIdx, mDrakeModel->FindBodyIndex("rKnee")).linear() << std::endl;
    std::cout << "actual robot" << std::endl;
    std::cout << mRobot->getBodyNodeIsometry("lKnee").linear() << std::endl;

    //std::cout << "[CoM Velocity]" << std::endl;
    //std::cout << "desired" << std::endl;
    //std::cout << mCoMVelDes << std::endl;
    //std::cout << "sol" << std::endl;
    //std::cout << dartModel.getCoMJacobian().block(3, 0, 3, mRobot->getNumActuatedDofs()) * qdot<< std::endl;
    //std::cout << "actual robot" << std::endl;
    //std::cout << mRobot->getCoMJacobian().block(3, 0, 3, mRobot->getNumActuatedDofs()) * mRobot->getQ() << std::endl;

    //std::cout << "[Right Foot Velocity]" << std::endl;
    //std::cout << "sol" << std::endl;
    //std::cout << dartModel.getBodyNodeJacobian("rAnkle") * qdot << std::endl;
    //std::cout << "actual robot" << std::endl;
    //std::cout << mRobot->getBodyNodeJacobian("rAnkle") * mRobot->getQdot() << std::endl;

    //std::cout << "[Left Foot Velocity]" << std::endl;
    //std::cout << "sol" << std::endl;
    //std::cout << dartModel.getBodyNodeJacobian("lAnkle") * qdot << std::endl;
    //std::cout << "actual robot" << std::endl;
    //std::cout << mRobot->getBodyNodeJacobian("lAnkle") * mRobot->getQdot() << std::endl;
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
    std::cout << mMid[0] << " --> " << (mInitRfIso.translation())[0] << std::endl;
    mMid[0] = (mInitRfIso.translation())[0];
    //mMid[2] -= 0.05;
    //mAmp[2] = 0.05;
    mAmp[2] = 0.0;
    mFreq[2] = 0.6;
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
