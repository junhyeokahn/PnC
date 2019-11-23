#include "gtest/gtest.h"

#include <Configuration.h>
#include <Eigen/Dense>
#include <Utils/IO/IOUtilities.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/WBC/IHWBC/IHWBC.hpp>

// Draco Specific
#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/DracoDefinition.hpp>

void setInitialConfiguration(RobotSystem* & robot) {
    int lKneeIdx = robot->getDofIdx("lKnee");
    int lHipPitchIdx = robot->getDofIdx("lHipPitch");
    int rKneeIdx = robot->getDofIdx("rKnee");
    int rHipPitchIdx = robot->getDofIdx("rHipPitch");
    int lAnkleIdx = robot->getDofIdx("lAnkle");
    int rAnkleIdx = robot->getDofIdx("rAnkle");

    Eigen::VectorXd q = robot->getQ();
    Eigen::VectorXd qdot(robot->getNumDofs()); 
    qdot.setZero();

    q[2] = 1.193;
    double alpha(-M_PI / 4.);
    double beta(M_PI / 5.5);
    q[lHipPitchIdx] = alpha;
    q[lKneeIdx] = beta - alpha;
    q[rHipPitchIdx] = alpha;
    q[rKneeIdx] = beta - alpha;
    q[lAnkleIdx] = M_PI / 2 - beta;
    q[rAnkleIdx] = M_PI / 2 - beta;

    robot->updateSystem(q, qdot);
}


TEST(IHWBC, robot) {
    RobotSystem* robot;
    robot = new RobotSystem(6, THIS_COM "RobotModel/Robot/Draco/DracoPnC_Dart.urdf");

    //  Initialize actuated list
    std::vector<bool> act_list;
    act_list.resize(robot->getNumDofs(), true);
    for (int i(0); i < robot->getNumVirtualDofs(); ++i){
	    act_list[i] = false;	
    } 

    // Initialize IHWBC
    IHWBC* ihwbc = new IHWBC(act_list);

    // Task and Contact list
	std::vector<Task*> task_list;
	std::vector<ContactSpec*> contact_list;

    // Create Tasks
    Task* total_joint_task = new BasicTask(robot, BasicTaskType::JOINT, Draco::n_adof);
    task_list.push_back(total_joint_task);

    // Create the contacts
    ContactSpec* rfoot_front_contact = new PointContactSpec(robot, DracoBodyNode::rFootFront, 0.7);
    ContactSpec* rfoot_back_contact = new PointContactSpec(robot, DracoBodyNode::rFootBack, 0.7);
    ContactSpec* lfoot_front_contact = new PointContactSpec(robot, DracoBodyNode::lFootFront, 0.7);
    ContactSpec* lfoot_back_contact = new PointContactSpec(robot, DracoBodyNode::lFootBack, 0.7);

    contact_list.push_back(rfoot_front_contact);
    contact_list.push_back(rfoot_back_contact);
    contact_list.push_back(lfoot_front_contact);
    contact_list.push_back(lfoot_back_contact);

    setInitialConfiguration(robot);
    std::cout << "q:" << robot->getQ().transpose() << std::endl;

	Eigen::MatrixXd A = robot->getMassMatrix();
	Eigen::MatrixXd Ainv = robot->getInvMassMatrix();
	Eigen::MatrixXd grav = robot->getGravity();
	Eigen::MatrixXd coriolis = robot->getCoriolis();

	// myUtils::pretty_print(A, std::cout, "A");
}

