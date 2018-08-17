#include "DracoHipPnC/TestSet/TestSet.hpp"
#include "RobotSystem.hpp"
#include "Utilities.hpp"

GravityCompensationTest::GravityCompensationTest(RobotSystem* robot_): Test(robot_) {
    // Choose Planner

    // Choose Controller
    printf("[Gravity Compensation Test] Constructed\n");
}

GravityCompensationTest::~GravityCompensationTest() {
}

void GravityCompensationTest::getTorqueInput(void * commandData_) {

}

void GravityCompensationTest::initialize() {
    //Planner Initialize

    //Controller Initialize

    isInitialized = true;
}
