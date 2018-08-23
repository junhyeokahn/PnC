#include "DracoPnC/TestSet/TestSet.hpp"
#include "RobotSystem.hpp"
#include "Utilities.hpp"
#include <MathmaticaDraco.h>

CentroidKinematicOptimizationTest::CentroidKinematicOptimizationTest(RobotSystem* robot_): Test(robot_) {
    // Choose Planner

    // Choose Controller
    printf("[Centroid Kinematic Optimization Test] Constructed\n");
}

CentroidKinematicOptimizationTest::~CentroidKinematicOptimizationTest() {
}

void CentroidKinematicOptimizationTest::getTorqueInput(void * commandData_) {
}

void CentroidKinematicOptimizationTest::initialize() {
    //Planner Initialize

    //Controller Initialize

    isInitialized = true;
}
