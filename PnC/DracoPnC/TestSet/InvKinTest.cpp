#include "DracoPnC/TestSet/TestSet.hpp"
#include "RobotSystem.hpp"
#include "Utilities.hpp"

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
    printf("[Inv Kin Test] Constructed\n");
}

InvKinTest::~InvKinTest() {
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
    double posTol(0.001);
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
    Eigen::Vector3d comLb = mInitCOM - Eigen::Vector3d::Constant(posTol);
    Eigen::Vector3d comUb = mInitCOM + Eigen::Vector3d::Constant(posTol);
    //comLb[2] -= 0.2;
    //comUb[2] -= 0.2;
    WorldCoMConstraint CoMc(mDrakeModel.get(), comLb, comUb, tspan);
    constraint_array.push_back(&CoMc);

    const IKoptions ikoptions(mDrakeModel.get());
    Eigen::VectorXd qSol = Eigen::VectorXd::Zero(mDrakeModel->get_num_positions());
    int info = 0;
    std::vector<std::string> infeasible_constraint;
    inverseKin(mDrakeModel.get(), mInitQ, mInitQ, constraint_array.size(),
            constraint_array.data(), ikoptions, &qSol, &info,
            &infeasible_constraint);
    if (info == 1) {
        cmd->q = qSol;
        cmd->qdot.setZero();
        cmd->jtrq.setZero();

        std::cout << "----------Validation--------------" << std::endl;
        KinematicsCache<double> cache = mDrakeModel->doKinematics(qSol);
        auto rfiso_res = mDrakeModel->relativeTransform(cache, 0, mRfIdx);
        auto lfiso_res = mDrakeModel->relativeTransform(cache, 0, mLfIdx);
        auto com_res = mDrakeModel->centerOfMass(cache);
        std::cout << "rfootDes" << std::endl;
        std::cout << mInitRfIso.translation() << std::endl;
        std::cout << "rfootSol" << std::endl;
        std::cout << rfiso_res.translation() << std::endl;
        std::cout << "===" << std::endl;
        std::cout << "lfootDes" << std::endl;
        std::cout << mInitLfIso.translation() << std::endl;
        std::cout << "lfootSol" << std::endl;
        std::cout << lfiso_res.translation() << std::endl;
        std::cout << "===" << std::endl;
        std::cout << "comDes" << std::endl;
        std::cout << mInitCOM << std::endl;
        std::cout << "comSol" << std::endl;
        std::cout << com_res << std::endl;

    } else {
        std::cout << "[Inverse Kinematic Solver Falied] INFO : " << info << std::endl;
        exit(0);
    }
}

void InvKinTest::initialize() {
    //Planner Initialize
    mInitQ = mRobot->getInitialConfiguration();
    KinematicsCache<double> cache = mDrakeModel->doKinematics(mInitQ);
    mInitRfIso = mDrakeModel->relativeTransform(cache, mWorldIdx, mRfIdx);
    mInitLfIso = mDrakeModel->relativeTransform(cache, mWorldIdx, mLfIdx);
    mInitCOM = mDrakeModel->centerOfMass(cache);

    //Controller Initialize

    isInitialized = true;
}
