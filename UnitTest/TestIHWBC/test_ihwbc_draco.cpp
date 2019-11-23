#include "gtest/gtest.h"

#include <Configuration.h>
#include <Eigen/Dense>
#include <Utils/IO/IOUtilities.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/WBC/IHWBC/IHWBC.hpp>

// Draco Specific
#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/DracoPnC/DracoDefinition.hpp>

void getInitialConfiguration(RobotSystem* & robot, Eigen::VectorXd & q_out, Eigen::VectorXd & qdot_out) {
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

    q_out = q;
    qdot_out = qdot;
}

// Test 3 types of IHWBC:
//   1) No target reaction force minimization
//   2) Term-by-term minimization minimization
//   3) Desired Contact Wrench minimization

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

    // Initialize and Update the robot
    Eigen::VectorXd q, qdot;
    getInitialConfiguration(robot, q, qdot);
    robot->updateSystem(q, qdot);
    myUtils::pretty_print(q, std::cout, "q");    

    // Task and Contact list
	std::vector<Task*> task_list;
	std::vector<ContactSpec*> contact_list;

    // Create Tasks

	// Body RxRyZ tasks or CoM xyz task or Linear Momentum Task
	// Body Rx Ry Rz
	// Foot location task.
    Task* body_rpz_task_ = new BodyRxRyZTask(robot);
    Task* rfoot_center_rz_xyz_task = new FootRzXYZTask(robot, DracoBodyNode::rFootCenter);
    Task* lfoot_center_rz_xyz_task = new FootRzXYZTask(robot, DracoBodyNode::lFootCenter);
    Task* total_joint_task = new BasicTask(robot, BasicTaskType::JOINT, Draco::n_adof);

    task_list.push_back(total_joint_task);

    // Set Task Gains
    Eigen::VectorXd kp_jp = Eigen::VectorXd::Ones(Draco::n_adof); 
    Eigen::VectorXd kd_jp = 0.1*Eigen::VectorXd::Ones(Draco::n_adof);
    total_joint_task->setGain(kp_jp, kd_jp);

    // Create the contacts
    ContactSpec* rfoot_front_contact = new PointContactSpec(robot, DracoBodyNode::rFootFront, 0.7);
    ContactSpec* rfoot_back_contact = new PointContactSpec(robot, DracoBodyNode::rFootBack, 0.7);
    ContactSpec* lfoot_front_contact = new PointContactSpec(robot, DracoBodyNode::lFootFront, 0.7);
    ContactSpec* lfoot_back_contact = new PointContactSpec(robot, DracoBodyNode::lFootBack, 0.7);

    contact_list.push_back(rfoot_front_contact);
    contact_list.push_back(rfoot_back_contact);
    contact_list.push_back(lfoot_front_contact);
    contact_list.push_back(lfoot_back_contact);

	// Set the desired task values
   	Eigen::VectorXd jpos_des = 0.95*robot->getQ().tail(Draco::n_adof);
    Eigen::VectorXd jvel_des(Draco::n_adof);  jvel_des.setZero();
    Eigen::VectorXd jacc_des(Draco::n_adof);  jacc_des.setZero();
    total_joint_task->updateTask(jpos_des, jvel_des, jacc_des);

    myUtils::pretty_print(jpos_des, std::cout, "jpos_des");

    // Update dynamics
	Eigen::MatrixXd A = robot->getMassMatrix();
	Eigen::MatrixXd Ainv = robot->getInvMassMatrix();
	Eigen::MatrixXd grav = robot->getGravity();
	Eigen::MatrixXd coriolis = robot->getCoriolis();

	// Containers
	Eigen::VectorXd tau_cmd(robot->getNumDofs() - robot->getNumVirtualDofs()); // Torque Command output from IHBC
	Eigen::VectorXd qddot_cmd(robot->getNumDofs() - robot->getNumVirtualDofs()); // Joint Acceleration Command output from IHBC	

	int contact_dim_size = 0;
    for(int i = 0; i < contact_list.size(); i++){
    	contact_dim_size += contact_list[i]->getDim();
    }
    ASSERT_EQ(contact_dim_size, 12);

    Eigen::VectorXd Fd(contact_dim_size); Fd.setZero();

    // Initialize QP weights
 	Eigen::VectorXd w_task_heirarchy(task_list.size());  // Vector of task priority weighs
 	w_task_heirarchy[0] = 1e-3;
 	double w_contact_weight = 2.0; // Contact Weight

 	double lambda_qddot = 1e-16;
 	double lambda_Fr = 1e-16;

    // Set QP weights
 	ihwbc->setQPWeights(w_task_heirarchy, w_contact_weight);
 	ihwbc->setRegularizationTerms(lambda_qddot, lambda_Fr);

 	// Update and solve QP
 	ihwbc->updateSetting(A, Ainv, coriolis, grav);
    ihwbc->solve(task_list, contact_list, Fd, tau_cmd, qddot_cmd);

	// myUtils::pretty_print(A, std::cout, "A");
}

