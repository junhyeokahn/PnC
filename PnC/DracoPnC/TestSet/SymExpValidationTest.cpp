#include "DracoPnC/TestSet/TestSet.hpp"
#include "RobotSystem.hpp"
#include "Utilities.hpp"
#include <MathmaticaDracoFixed.h>

SymExpValidationTest::SymExpValidationTest(RobotSystem* robot_): Test(robot_) {
    // Choose Planner

    // Choose Controller
    printf("[Sym Exp Validation Test] Constructed\n");
}

SymExpValidationTest::~SymExpValidationTest() {
}

void SymExpValidationTest::getTorqueInput(void * commandData_) {
    DracoCommand* cmd = (DracoCommand*) commandData_;
    cmd->q = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->qdot = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->jtrq = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());

    // Comparison to Mathematica
    Eigen::Vector3d lHipYaw1, lHipRoll1, lHipPitch1, lKnee1, lAnkle1, rHipYaw1, rHipRoll1, rHipPitch1, rKnee1, rAnkle1, com1;
    Eigen::Vector3d lHipYaw2, lHipRoll2, lHipPitch2, lKnee2, lAnkle2, rHipYaw2, rHipRoll2, rHipPitch2, rKnee2, rAnkle2, com2;

    Eigen::VectorXd zero_vector = Eigen::VectorXd::Zero(16);
    p_lHipYaw(lHipYaw1, zero_vector);
    lHipYaw2 = mRobot->getBodyNodeIsometry("lHipYaw").translation();
    static Eigen::Vector3d offset = lHipYaw1 - lHipYaw2;

    p_lHipRoll(lHipRoll1, zero_vector);
    p_lHipPitch(lHipPitch1, zero_vector);
    p_lKnee(lKnee1, zero_vector);
    p_lAnkle(lAnkle1, zero_vector);
    p_rHipYaw(rHipYaw1, zero_vector);
    p_rHipRoll(rHipRoll1, zero_vector);
    p_rHipPitch(rHipPitch1, zero_vector);
    p_rKnee(rKnee1, zero_vector);
    p_rAnkle(rAnkle1, zero_vector);
    pcom_Draco(com1, zero_vector);

    lHipRoll2 = mRobot->getBodyNodeIsometry("lHipRoll").translation();
    lHipPitch2 = mRobot->getBodyNodeIsometry("lHipPitch").translation();
    lKnee2 = mRobot->getBodyNodeIsometry("lKnee").translation();
    lAnkle2 = mRobot->getBodyNodeIsometry("lAnkle").translation();
    rHipYaw2 = mRobot->getBodyNodeIsometry("rHipYaw").translation();
    rHipRoll2 = mRobot->getBodyNodeIsometry("rHipRoll").translation();
    rHipPitch2 = mRobot->getBodyNodeIsometry("rHipPitch").translation();
    rKnee2 = mRobot->getBodyNodeIsometry("rKnee").translation();
    rAnkle2 = mRobot->getBodyNodeIsometry("rAnkle").translation();
    com2 = mRobot->getCoMPosition();

    if (!(myUtils::isEqual(lHipRoll1, lHipRoll2))) std::cout << "left hip roll is not equal" << std::endl;
    if (!(myUtils::isEqual(lHipPitch1, lHipPitch2))) std::cout << "left hip pitch is not equal" << std::endl;
    if (!(myUtils::isEqual(lKnee1, lKnee2))) std::cout << "left knee is not equal" << std::endl;
    if (!(myUtils::isEqual(lAnkle1, lAnkle2))) std::cout << "left ankle is not equal" << std::endl;
    if (!(myUtils::isEqual(rHipRoll1, rHipRoll2))) std::cout << "right hip roll is not equal" << std::endl;
    if (!(myUtils::isEqual(rHipPitch1, rHipPitch2))) std::cout << "right hip pitch is not equal" << std::endl;
    if (!(myUtils::isEqual(rKnee1, rKnee2))) std::cout << "right knee is not equal" << std::endl;
    if (!(myUtils::isEqual(rAnkle1, rAnkle2))) std::cout << "right ankle is not equal" << std::endl;
    if (!(myUtils::isEqual(com1, com2))) std::cout << "com is not equal" << std::endl;
    exit(0);

}

void SymExpValidationTest::initialize() {
    //Planner Initialize

    //Controller Initialize

    isInitialized = true;
}
