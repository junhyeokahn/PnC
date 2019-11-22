#include <Configuration.h>
#include <Eigen/Dense>
#include <Utils/IO/IOUtilities.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include "gtest/gtest.h"

TEST(IHWBC, robot) {
    RobotSystem* robot;
    robot = new RobotSystem(6, THIS_COM "RobotModel/Robot/Draco/DracoPnC_Dart.urdf");
    std::cout << "q:" << robot->getQ().transpose() << std::endl;
}


// int main(int argc, char ** argv){
//     RobotSystem* robot;
//     robot = new RobotSystem(6, THIS_COM "RobotModel/Robot/Draco/DracoPnC_Dart.urdf");
//     std::cout << "q:" << robot->getQ().transpose() << std::endl;
// }	
// #include <dart/dart.hpp>
// #include <dart/gui/osg/osg.hpp>
// #include <dart/utils/urdf/urdf.hpp>
// #include <dart/utils/utils.hpp>

// void setInitialConfiguration(dart::dynamics::SkeletonPtr & robot) {
//     int lKneeIdx = robot->getDof("lKnee")->getIndexInSkeleton();
//     int lHipPitchIdx = robot->getDof("lHipPitch")->getIndexInSkeleton();
//     int rKneeIdx = robot->getDof("rKnee")->getIndexInSkeleton();
//     int rHipPitchIdx = robot->getDof("rHipPitch")->getIndexInSkeleton();
//     int lAnkleIdx = robot->getDof("lAnkle")->getIndexInSkeleton();
//     int rAnkleIdx = robot->getDof("rAnkle")->getIndexInSkeleton();

//     int initPos(2);  // 0 : Home, 1 : Simulation, 2 : Experiment
//     Eigen::VectorXd q = robot->getPositions();

//     q[2] = 1.193;
//     double alpha(-M_PI / 4.);
//     double beta(M_PI / 5.5);
//     q[lHipPitchIdx] = alpha;
//     q[lKneeIdx] = beta - alpha;
//     q[rHipPitchIdx] = alpha;
//     q[rKneeIdx] = beta - alpha;
//     q[lAnkleIdx] = M_PI / 2 - beta;
//     q[rAnkleIdx] = M_PI / 2 - beta;

//     robot->setPositions(q);
// }

// int main(int argc, char ** argv){
//     dart::utils::DartLoader urdfLoader;
//     dart::dynamics::SkeletonPtr robot = urdfLoader.parseSkeleton(
//         THIS_COM "RobotModel/Robot/Draco/DracoSim_Dart.urdf");
// 	setInitialConfiguration(robot);


//     std::cout << "robot position" << std::endl;
//     std::cout << robot->getPositions().transpose() << std::endl;
// }