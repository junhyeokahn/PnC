#include "DracoHipPnC/TestSet/TestSet.hpp"
#include "RobotSystem.hpp"
#include "Utilities.hpp"
#include <MathmaticaDracoFixed.h>

GravityCompensationTest::GravityCompensationTest(RobotSystem* robot_): Test(robot_) {
    // Choose Planner

    // Choose Controller
    printf("[Gravity Compensation Test] Constructed\n");
}

GravityCompensationTest::~GravityCompensationTest() {
}

void GravityCompensationTest::getTorqueInput(void * commandData_) {
    DracoHipCommand* cmd = (DracoHipCommand*) commandData_;
    cmd->q = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->qdot = Eigen::VectorXd::Zero(mRobot->getNumDofs());
    cmd->jtrq = Eigen::VectorXd::Zero(mRobot->getNumActuatedDofs());

    // Comparison to Mathematica
    Eigen::Vector3d lHipYaw1, lHipRoll1, lHipPitch1, lKnee1, lAnkle1, rHipYaw1, rHipRoll1, rHipPitch1, rKnee1, rAnkle1, com1;
    Eigen::Vector3d lHipYaw2, lHipRoll2, lHipPitch2, lKnee2, lAnkle2, rHipYaw2, rHipRoll2, rHipPitch2, rKnee2, rAnkle2, com2;

    p_lHipYaw(lHipYaw1, mRobot->getQ());
    lHipYaw2 = mRobot->getBodyNodeIsometry("lHipYaw").translation();
    static Eigen::Vector3d offset = lHipYaw1 - lHipYaw2;

    p_lHipRoll(lHipRoll1, mRobot->getQ());
    p_lHipPitch(lHipPitch1, mRobot->getQ());
    p_lKnee(lKnee1, mRobot->getQ());
    p_lAnkle(lAnkle1, mRobot->getQ());
    p_rHipYaw(rHipYaw1, mRobot->getQ());
    p_rHipRoll(rHipRoll1, mRobot->getQ());
    p_rHipPitch(rHipPitch1, mRobot->getQ());
    p_rKnee(rKnee1, mRobot->getQ());
    p_rAnkle(rAnkle1, mRobot->getQ());
    pcom_Draco(com1, mRobot->getQ());

    //lHipRoll2 = mRobot->getBodyNodeIsometry("lHipRoll").translation() + offset;
    //lHipPitch2 = mRobot->getBodyNodeIsometry("lHipPitch").translation() + offset;
    //lKnee2 = mRobot->getBodyNodeIsometry("lKnee").translation() + offset;
    //lAnkle2 = mRobot->getBodyNodeIsometry("lAnkle").translation() + offset;
    //rHipYaw2 = mRobot->getBodyNodeIsometry("rHipYaw").translation() + offset;
    //rHipRoll2 = mRobot->getBodyNodeIsometry("rHipRoll").translation() + offset;
    //rHipPitch2 = mRobot->getBodyNodeIsometry("rHipPitch").translation() + offset;
    //rKnee2 = mRobot->getBodyNodeIsometry("rKnee").translation() + offset;
    //rAnkle2 = mRobot->getBodyNodeIsometry("rAnkle").translation() + offset;
    //com2 = mRobot->getCoMPosition() + offset;

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

    //std::cout << "++++++++++++++++++++++++" << std::endl;
    //std::cout << lHipRoll1 - lHipRoll2 << std::endl;
    //std::cout << "---" << std::endl;
    //std::cout << lHipPitch1 - lHipPitch2 << std::endl;
    //std::cout << "---" << std::endl;
    //std::cout << lKnee1 - lKnee2 << std::endl;
    //std::cout << "---" << std::endl;
    //std::cout << lAnkle1 - lAnkle2 << std::endl;
    //std::cout << "000" << std::endl;
    //std::cout << rHipRoll1 - rHipRoll2 << std::endl;
    //std::cout << "---" << std::endl;
    //std::cout << rHipPitch1 - rHipPitch2 << std::endl;
    //std::cout << "---" << std::endl;
    //std::cout << rKnee1 - rKnee2 << std::endl;
    //std::cout << "---" << std::endl;
    //std::cout << rAnkle1 - rAnkle2 << std::endl;


    std::cout << lHipYaw1 << std::endl;
    std::cout << "------" << std::endl;
    std::cout << lHipYaw2 << std::endl;


    //if (!(myUtils::isEqual(lHipRoll1, lHipRoll2))) std::cout << "left hip roll is not equal" << std::endl;
    //if (!(myUtils::isEqual(lHipPitch1, lHipPitch2))) std::cout << "left hip pitch is not equal" << std::endl;
    //if (!(myUtils::isEqual(lKnee1, lKnee2))) std::cout << "left knee is not equal" << std::endl;
    //if (!(myUtils::isEqual(lAnkle1, lAnkle2))) std::cout << "left ankle is not equal" << std::endl;
    //if (!(myUtils::isEqual(rHipRoll1, rHipRoll2))) std::cout << "right hip roll is not equal" << std::endl;
    //if (!(myUtils::isEqual(rHipPitch1, rHipPitch2))) std::cout << "right hip pitch is not equal" << std::endl;
    //if (!(myUtils::isEqual(rKnee1, rKnee2))) std::cout << "right knee is not equal" << std::endl;
    //if (!(myUtils::isEqual(rAnkle1, rAnkle2))) std::cout << "right ankle is not equal" << std::endl;
    //if (!(myUtils::isEqual(com1, com2))) std::cout << "com is not equal" << std::endl;
    //std::cout << rHipRoll1 - rHipRoll2 << std::endl;
    //std::cout << "--" << std::endl;
    //std::cout << rHipPitch1 - rHipPitch2 << std::endl;
    //std::cout << "--" << std::endl;
    //std::cout << rKnee1 - rKnee2 << std::endl;
    //std::cout << "--" << std::endl;
    //std::cout << rAnkle1 - rAnkle2 << std::endl;
    //std::cout << "--" << std::endl;
    //std::cout << offset << std::endl;
    ////exit(0);

}

void GravityCompensationTest::initialize() {
    //Planner Initialize

    //Controller Initialize

    isInitialized = true;
}
