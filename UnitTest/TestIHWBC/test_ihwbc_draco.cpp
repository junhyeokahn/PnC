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
//   2) Term-by-term force minimization
//   3) Term-by-term no body task
//   4) Desired Contact Wrench minimization

TEST(IHWBC, no_target_reaction_force) {
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
    // myUtils::pretty_print(q, std::cout, "q");    

    // Task and Contact list
    std::vector<Task*> task_list;
    std::vector<ContactSpec*> contact_list;

    // Create Tasks
    // Body RxRyZ tasks or CoM xyz task or Linear Momentum Task
    // Body Rx Ry Rz
    // Foot location task.
    // Joint Position Task. It appears that it's important to have this task to ensure that uncontrolled qddot does not blow up
    Task* body_rpz_task_ = new BodyRxRyZTask(robot);
    Task* rfoot_center_rz_xyz_task = new FootRzXYZTask(robot, DracoBodyNode::rFootCenter);
    Task* lfoot_center_rz_xyz_task = new FootRzXYZTask(robot, DracoBodyNode::lFootCenter);
    Task* total_joint_task = new BasicTask(robot, BasicTaskType::JOINT, Draco::n_adof);

    task_list.push_back(body_rpz_task_);
    task_list.push_back(rfoot_center_rz_xyz_task);
    task_list.push_back(lfoot_center_rz_xyz_task);
    task_list.push_back(total_joint_task);

    // Set Task Gains
    Eigen::VectorXd kp_body = 100*Eigen::VectorXd::Ones(3); 
    Eigen::VectorXd kd_body = 1.0*Eigen::VectorXd::Ones(3);
    body_rpz_task_->setGain(kp_body, kd_body);

    Eigen::VectorXd kp_foot = 100*Eigen::VectorXd::Ones(4); 
    Eigen::VectorXd kd_foot = 1.0*Eigen::VectorXd::Ones(4);
    rfoot_center_rz_xyz_task->setGain(kp_foot, kd_foot);
    lfoot_center_rz_xyz_task->setGain(kp_foot, kd_foot);

    Eigen::VectorXd kp_jp = Eigen::VectorXd::Ones(Draco::n_adof); 
    Eigen::VectorXd kd_jp = 0.1*Eigen::VectorXd::Ones(Draco::n_adof);
    total_joint_task->setGain(kp_jp, kd_jp);


    // Set the desired task values
    // Body Task
    Eigen::VectorXd body_pos_des(7); body_pos_des.setZero();
    Eigen::VectorXd body_vel_des(6); body_vel_des.setZero();    
    Eigen::VectorXd body_acc_des(6); body_acc_des.setZero();
    Eigen::Quaternion<double> body_des_quat(1, 0, 0, 0);

    body_pos_des[0] = body_des_quat.w();
    body_pos_des[1] = body_des_quat.x();
    body_pos_des[2] = body_des_quat.y();
    body_pos_des[3] = body_des_quat.z();

    body_pos_des[4] = q[0];
    body_pos_des[5] = q[1];
    body_pos_des[6] = q[2];// + 0.02;
    
    body_rpz_task_->updateTask(body_pos_des, body_vel_des, body_acc_des);

    // Foot Task
    Eigen::VectorXd rfoot_pos_des(7); rfoot_pos_des.setZero();
    Eigen::VectorXd lfoot_pos_des(7); lfoot_pos_des.setZero();
    Eigen::VectorXd foot_vel_des(6); foot_vel_des.setZero();    
    Eigen::VectorXd foot_acc_des(6); foot_acc_des.setZero();

    Eigen::Quaternion<double> rfoot_ori_act(robot->getBodyNodeCoMIsometry(DracoBodyNode::rFootCenter).linear());
    Eigen::Quaternion<double> lfoot_ori_act(robot->getBodyNodeCoMIsometry(DracoBodyNode::lFootCenter).linear());

    rfoot_pos_des[0] = rfoot_ori_act.w();
    rfoot_pos_des[1] = rfoot_ori_act.x();
    rfoot_pos_des[2] = rfoot_ori_act.y();
    rfoot_pos_des[3] = rfoot_ori_act.z();
    rfoot_pos_des.tail(3) = robot->getBodyNodeCoMIsometry(DracoBodyNode::rFootCenter).translation();

    lfoot_pos_des[0] = lfoot_ori_act.w();
    lfoot_pos_des[1] = lfoot_ori_act.x();
    lfoot_pos_des[2] = lfoot_ori_act.y();
    lfoot_pos_des[3] = lfoot_ori_act.z();
    lfoot_pos_des.tail(3) = robot->getBodyNodeCoMIsometry(DracoBodyNode::lFootCenter).translation();

    // Joint Position Task
    Eigen::VectorXd jpos_des = 0.95*robot->getQ().tail(Draco::n_adof);
    Eigen::VectorXd jvel_des(Draco::n_adof);  jvel_des.setZero();
    Eigen::VectorXd jacc_des(Draco::n_adof);  jacc_des.setZero();
    total_joint_task->updateTask(jpos_des, jvel_des, jacc_des);

    // myUtils::pretty_print(body_pos_des, std::cout, "body_pos_des");
    // myUtils::pretty_print(rfoot_pos_des, std::cout, "rfoot_pos_des");
    // myUtils::pretty_print(lfoot_pos_des, std::cout, "lfoot_pos_des");
    // myUtils::pretty_print(jpos_des, std::cout, "jpos_des");

    // Create the contacts
    ContactSpec* rfoot_front_contact = new PointContactSpec(robot, DracoBodyNode::rFootFront, 0.7);
    ContactSpec* rfoot_back_contact = new PointContactSpec(robot, DracoBodyNode::rFootBack, 0.7);
    ContactSpec* lfoot_front_contact = new PointContactSpec(robot, DracoBodyNode::lFootFront, 0.7);
    ContactSpec* lfoot_back_contact = new PointContactSpec(robot, DracoBodyNode::lFootBack, 0.7);

    contact_list.push_back(rfoot_front_contact);
    contact_list.push_back(rfoot_back_contact);
    contact_list.push_back(lfoot_front_contact);
    contact_list.push_back(lfoot_back_contact);

    // ((PointContactSpec*)rfoot_front_contact)->setMaxFz(100.0);

    rfoot_front_contact->updateContactSpec();
    rfoot_back_contact->updateContactSpec();
    lfoot_front_contact->updateContactSpec();
    lfoot_back_contact->updateContactSpec();

    // Update dynamics
    Eigen::MatrixXd A = robot->getMassMatrix();
    Eigen::MatrixXd Ainv = robot->getInvMassMatrix();
    Eigen::VectorXd grav = robot->getGravity();
    Eigen::MatrixXd coriolis = robot->getCoriolis();

    // myUtils::pretty_print(A, std::cout, "A");

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

    // Contact Tasks have weight 1.0
    // Pose Tasks have weight 1e-4
    // Joint Posture Tasks have weight 1e-6
    w_task_heirarchy[0] = 1e-4; // Body
    w_task_heirarchy[1] = 1.0; // rfoot
    w_task_heirarchy[2] = 1.0; // lfoot
    w_task_heirarchy[3] = 1e-6; // joint

    // When Fd is zero, the contact weight should be smaller than the 
    // tasks to encourage using the reaction forces instead of qddot 
    // for satisfying the floating base dynamics.
    double w_contact_weight = 1e-10; // Contact Weight

    // Regularization terms should always be the lowest cost. 
    double lambda_qddot = 1e-16;
    double lambda_Fr = 1e-16;

    // Set QP weights
    ihwbc->setQPWeights(w_task_heirarchy, w_contact_weight);
    ihwbc->setRegularizationTerms(lambda_qddot, lambda_Fr);

    // Update and solve QP
    ihwbc->updateSetting(A, Ainv, coriolis, grav);
    ihwbc->solve(task_list, contact_list, Fd, tau_cmd, qddot_cmd);

    // QP dec variable results
    Eigen::VectorXd qddot_res;
    Eigen::VectorXd Fr_res;

    ihwbc->getQddotResult(qddot_res);
    ihwbc->getFrResult(Fr_res);

    double total_Fz = 0;
    int idx_offset = 0;
    for(int i = 0; i < contact_list.size(); i++){
        total_Fz += Fr_res[idx_offset + contact_list[i]->getFzIndex()];
        idx_offset += contact_list[i]->getDim();
    }

    myUtils::pretty_print(Fd, std::cout, "Fd");
    myUtils::pretty_print(tau_cmd, std::cout, "tau_cmd");
    myUtils::pretty_print(qddot_cmd, std::cout, "qddot_cmd");
    myUtils::pretty_print(qddot_res, std::cout, "qddot_res");
    myUtils::pretty_print(Fr_res, std::cout, "Fr_res");
    std::cout << "robot mass = " << robot->getRobotMass() << std::endl;
    std::cout << "robot mass*grav = " << robot->getRobotMass()*9.81 << std::endl;
    std::cout << "total_Fz:" << total_Fz << std::endl;

    // Assert that computed reaction forces must be less than 1 Newton
    double diff_Fz = robot->getRobotMass()*9.81 - total_Fz;
    ASSERT_LE(std::fabs(diff_Fz), 1.0);
}


// If we trust the MPC, we don't need a com/body task. As the reaction force will dictate the behavior
// However, if the com/body tasks are available, then the weights between the com/body task vs the contact force tracking will dictate
// the final output.
TEST(IHWBC, term_by_term_rf_computation) {
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
    // myUtils::pretty_print(q, std::cout, "q");    

    // Task and Contact list
    std::vector<Task*> task_list;
    std::vector<ContactSpec*> contact_list;

    // Create Tasks

    // Body RxRyZ tasks or CoM xyz task or Linear Momentum Task
    // Body Rx Ry Rz
    // Foot location task.
    // Joint Position Task. It appears that it's important to have this task to ensure that uncontrolled qddot does not blow up
    Task* body_rpz_task_ = new BodyRxRyZTask(robot);
    Task* rfoot_center_rz_xyz_task = new FootRzXYZTask(robot, DracoBodyNode::rFootCenter);
    Task* lfoot_center_rz_xyz_task = new FootRzXYZTask(robot, DracoBodyNode::lFootCenter);
    Task* total_joint_task = new BasicTask(robot, BasicTaskType::JOINT, Draco::n_adof);

    task_list.push_back(body_rpz_task_);
    task_list.push_back(rfoot_center_rz_xyz_task);
    task_list.push_back(lfoot_center_rz_xyz_task);
    task_list.push_back(total_joint_task);

    // Set Task Gains
    Eigen::VectorXd kp_body = 100*Eigen::VectorXd::Ones(3); 
    Eigen::VectorXd kd_body = 1.0*Eigen::VectorXd::Ones(3);
    body_rpz_task_->setGain(kp_body, kd_body);

    Eigen::VectorXd kp_foot = 100*Eigen::VectorXd::Ones(4); 
    Eigen::VectorXd kd_foot = 1.0*Eigen::VectorXd::Ones(4);
    rfoot_center_rz_xyz_task->setGain(kp_foot, kd_foot);
    lfoot_center_rz_xyz_task->setGain(kp_foot, kd_foot);

    Eigen::VectorXd kp_jp = Eigen::VectorXd::Ones(Draco::n_adof); 
    Eigen::VectorXd kd_jp = 0.1*Eigen::VectorXd::Ones(Draco::n_adof);
    total_joint_task->setGain(kp_jp, kd_jp);


    // Set the desired task values
    // Body Task
    Eigen::VectorXd body_pos_des(7); body_pos_des.setZero();
    Eigen::VectorXd body_vel_des(6); body_vel_des.setZero();    
    Eigen::VectorXd body_acc_des(6); body_acc_des.setZero();
    Eigen::Quaternion<double> body_des_quat(1, 0, 0, 0);

    body_pos_des[0] = body_des_quat.w();
    body_pos_des[1] = body_des_quat.x();
    body_pos_des[2] = body_des_quat.y();
    body_pos_des[3] = body_des_quat.z();

    body_pos_des[4] = q[0];
    body_pos_des[5] = q[1];
    body_pos_des[6] = q[2];
    
    body_rpz_task_->updateTask(body_pos_des, body_vel_des, body_acc_des);

    // Foot Task
    Eigen::VectorXd rfoot_pos_des(7); rfoot_pos_des.setZero();
    Eigen::VectorXd lfoot_pos_des(7); lfoot_pos_des.setZero();
    Eigen::VectorXd foot_vel_des(6); foot_vel_des.setZero();    
    Eigen::VectorXd foot_acc_des(6); foot_acc_des.setZero();

    Eigen::Quaternion<double> rfoot_ori_act(robot->getBodyNodeCoMIsometry(DracoBodyNode::rFootCenter).linear());
    Eigen::Quaternion<double> lfoot_ori_act(robot->getBodyNodeCoMIsometry(DracoBodyNode::lFootCenter).linear());

    rfoot_pos_des[0] = rfoot_ori_act.w();
    rfoot_pos_des[1] = rfoot_ori_act.x();
    rfoot_pos_des[2] = rfoot_ori_act.y();
    rfoot_pos_des[3] = rfoot_ori_act.z();
    rfoot_pos_des.tail(3) = robot->getBodyNodeCoMIsometry(DracoBodyNode::rFootCenter).translation();

    lfoot_pos_des[0] = lfoot_ori_act.w();
    lfoot_pos_des[1] = lfoot_ori_act.x();
    lfoot_pos_des[2] = lfoot_ori_act.y();
    lfoot_pos_des[3] = lfoot_ori_act.z();
    lfoot_pos_des.tail(3) = robot->getBodyNodeCoMIsometry(DracoBodyNode::lFootCenter).translation();

    // Joint Position Task
    Eigen::VectorXd jpos_des = 0.95*robot->getQ().tail(Draco::n_adof);
    Eigen::VectorXd jvel_des(Draco::n_adof);  jvel_des.setZero();
    Eigen::VectorXd jacc_des(Draco::n_adof);  jacc_des.setZero();
    total_joint_task->updateTask(jpos_des, jvel_des, jacc_des);

    // myUtils::pretty_print(body_pos_des, std::cout, "body_pos_des");
    // myUtils::pretty_print(rfoot_pos_des, std::cout, "rfoot_pos_des");
    // myUtils::pretty_print(lfoot_pos_des, std::cout, "lfoot_pos_des");
    // myUtils::pretty_print(jpos_des, std::cout, "jpos_des");

    // Create the contacts
    ContactSpec* rfoot_front_contact = new PointContactSpec(robot, DracoBodyNode::rFootFront, 0.7);
    ContactSpec* rfoot_back_contact = new PointContactSpec(robot, DracoBodyNode::rFootBack, 0.7);
    ContactSpec* lfoot_front_contact = new PointContactSpec(robot, DracoBodyNode::lFootFront, 0.7);
    ContactSpec* lfoot_back_contact = new PointContactSpec(robot, DracoBodyNode::lFootBack, 0.7);

    contact_list.push_back(rfoot_front_contact);
    contact_list.push_back(rfoot_back_contact);
    contact_list.push_back(lfoot_front_contact);
    contact_list.push_back(lfoot_back_contact);

    // ((PointContactSpec*)rfoot_front_contact)->setMaxFz(100.0);

    rfoot_front_contact->updateContactSpec();
    rfoot_back_contact->updateContactSpec();
    lfoot_front_contact->updateContactSpec();
    lfoot_back_contact->updateContactSpec();

    // Update dynamics
    Eigen::MatrixXd A = robot->getMassMatrix();
    Eigen::MatrixXd Ainv = robot->getInvMassMatrix();
    Eigen::VectorXd grav = robot->getGravity();
    Eigen::MatrixXd coriolis = robot->getCoriolis();

    // myUtils::pretty_print(A, std::cout, "A");

    // Containers
    Eigen::VectorXd tau_cmd(robot->getNumDofs() - robot->getNumVirtualDofs()); // Torque Command output from IHBC
    Eigen::VectorXd qddot_cmd(robot->getNumDofs() - robot->getNumVirtualDofs()); // Joint Acceleration Command output from IHBC 

    int contact_dim_size = 0;
    for(int i = 0; i < contact_list.size(); i++){
        contact_dim_size += contact_list[i]->getDim();
    }
    ASSERT_EQ(contact_dim_size, 12);

    Eigen::VectorXd Fd(contact_dim_size); Fd.setZero();

    // Distribute reaction forces equally according to the mass of the robot
    double Fz_dist = (robot->getRobotMass()*9.81)/contact_list.size();
    int idx_offset = 0;
    for(int i = 0; i < contact_list.size(); i++){
        Fd[idx_offset + contact_list[i]->getFzIndex()] = Fz_dist;
        idx_offset += contact_list[i]->getDim();
    }

    // Initialize QP weights
    Eigen::VectorXd w_task_heirarchy(task_list.size());  // Vector of task priority weighs

    // Contact Tasks have weight 1.0
    // Pose Tasks have weight 1e-4
    // Joint Posture Tasks have weight 1e-6
    w_task_heirarchy[0] = 1e-4; // Body
    w_task_heirarchy[1] = 1.0; // rfoot
    w_task_heirarchy[2] = 1.0; // lfoot
    w_task_heirarchy[3] = 1e-6; // joint

    // w_task_heirarchy[0] = 1.0; // rfoot
    // w_task_heirarchy[1] = 1.0; // lfoot
    // w_task_heirarchy[2] = 1e-6; // joint

    // When Fd is nonzero, we need to make the contact weight large if we want to trust the output of the 
    // MPC or planner
    // double w_contact_weight = 1e-10; // Contact Weight
    double w_contact_weight = 1.0; // Contact Weight
    // double w_contact_weight = 1.0/(robot->getRobotMass()*9.81);

    // Regularization terms should always be the lowest cost. 
    double lambda_qddot = 1e-16;
    double lambda_Fr = 1e-16;

    // Set QP weights
    ihwbc->setQPWeights(w_task_heirarchy, w_contact_weight);
    ihwbc->setRegularizationTerms(lambda_qddot, lambda_Fr);

    // Update and solve QP
    ihwbc->updateSetting(A, Ainv, coriolis, grav);
    ihwbc->solve(task_list, contact_list, Fd, tau_cmd, qddot_cmd);

    // QP dec variable results
    Eigen::VectorXd qddot_res;
    Eigen::VectorXd Fr_res;

    ihwbc->getQddotResult(qddot_res);
    ihwbc->getFrResult(Fr_res);

    double total_Fz = 0;
    idx_offset = 0;
    for(int i = 0; i < contact_list.size(); i++){
        total_Fz += Fr_res[idx_offset + contact_list[i]->getFzIndex()];
        idx_offset += contact_list[i]->getDim();
    }

    myUtils::pretty_print(Fd, std::cout, "Fd");
    myUtils::pretty_print(tau_cmd, std::cout, "tau_cmd");
    myUtils::pretty_print(qddot_cmd, std::cout, "qddot_cmd");
    myUtils::pretty_print(qddot_res, std::cout, "qddot_res");
    myUtils::pretty_print(Fr_res, std::cout, "Fr_res");
    std::cout << "robot mass = " << robot->getRobotMass() << std::endl;
    std::cout << "robot mass*grav = " << robot->getRobotMass()*9.81 << std::endl;
    std::cout << "total_Fz:" << total_Fz << std::endl;

    // Assert that computed reaction forces must be less than 1 Newton
    double diff_Fz = robot->getRobotMass()*9.81 - total_Fz;
    ASSERT_LE(std::fabs(diff_Fz), 1.0);

}


// The body task is optional if we trust the MPC behavior
TEST(IHWBC, term_by_term_rf_computation_no_body_task) {
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
    // myUtils::pretty_print(q, std::cout, "q");    

    // Task and Contact list
    std::vector<Task*> task_list;
    std::vector<ContactSpec*> contact_list;

    // Create Tasks

    // Body RxRyZ tasks or CoM xyz task or Linear Momentum Task
    // Body Rx Ry Rz
    // Foot location task.
    // Joint Position Task. It appears that it's important to have this task to ensure that uncontrolled qddot does not blow up
    // Task* body_rpz_task_ = new BodyRxRyZTask(robot);
    Task* rfoot_center_rz_xyz_task = new FootRzXYZTask(robot, DracoBodyNode::rFootCenter);
    Task* lfoot_center_rz_xyz_task = new FootRzXYZTask(robot, DracoBodyNode::lFootCenter);
    Task* total_joint_task = new BasicTask(robot, BasicTaskType::JOINT, Draco::n_adof);

    // task_list.push_back(body_rpz_task_);
    task_list.push_back(rfoot_center_rz_xyz_task);
    task_list.push_back(lfoot_center_rz_xyz_task);
    task_list.push_back(total_joint_task);

    // Set Task Gains
    Eigen::VectorXd kp_body = 100*Eigen::VectorXd::Ones(3); 
    Eigen::VectorXd kd_body = 1.0*Eigen::VectorXd::Ones(3);
    // body_rpz_task_->setGain(kp_body, kd_body);

    Eigen::VectorXd kp_foot = 100*Eigen::VectorXd::Ones(4); 
    Eigen::VectorXd kd_foot = 1.0*Eigen::VectorXd::Ones(4);
    rfoot_center_rz_xyz_task->setGain(kp_foot, kd_foot);
    lfoot_center_rz_xyz_task->setGain(kp_foot, kd_foot);

    Eigen::VectorXd kp_jp = Eigen::VectorXd::Ones(Draco::n_adof); 
    Eigen::VectorXd kd_jp = 0.1*Eigen::VectorXd::Ones(Draco::n_adof);
    total_joint_task->setGain(kp_jp, kd_jp);


    // Set the desired task values
    // Body Task
    Eigen::VectorXd body_pos_des(7); body_pos_des.setZero();
    Eigen::VectorXd body_vel_des(6); body_vel_des.setZero();    
    Eigen::VectorXd body_acc_des(6); body_acc_des.setZero();
    Eigen::Quaternion<double> body_des_quat(1, 0, 0, 0);

    body_pos_des[0] = body_des_quat.w();
    body_pos_des[1] = body_des_quat.x();
    body_pos_des[2] = body_des_quat.y();
    body_pos_des[3] = body_des_quat.z();

    body_pos_des[4] = q[0];
    body_pos_des[5] = q[1];
    body_pos_des[6] = q[2];// + 0.02;
    
    // body_rpz_task_->updateTask(body_pos_des, body_vel_des, body_acc_des);

    // Foot Task
    Eigen::VectorXd rfoot_pos_des(7); rfoot_pos_des.setZero();
    Eigen::VectorXd lfoot_pos_des(7); lfoot_pos_des.setZero();
    Eigen::VectorXd foot_vel_des(6); foot_vel_des.setZero();    
    Eigen::VectorXd foot_acc_des(6); foot_acc_des.setZero();

    Eigen::Quaternion<double> rfoot_ori_act(robot->getBodyNodeCoMIsometry(DracoBodyNode::rFootCenter).linear());
    Eigen::Quaternion<double> lfoot_ori_act(robot->getBodyNodeCoMIsometry(DracoBodyNode::lFootCenter).linear());

    rfoot_pos_des[0] = rfoot_ori_act.w();
    rfoot_pos_des[1] = rfoot_ori_act.x();
    rfoot_pos_des[2] = rfoot_ori_act.y();
    rfoot_pos_des[3] = rfoot_ori_act.z();
    rfoot_pos_des.tail(3) = robot->getBodyNodeCoMIsometry(DracoBodyNode::rFootCenter).translation();

    lfoot_pos_des[0] = lfoot_ori_act.w();
    lfoot_pos_des[1] = lfoot_ori_act.x();
    lfoot_pos_des[2] = lfoot_ori_act.y();
    lfoot_pos_des[3] = lfoot_ori_act.z();
    lfoot_pos_des.tail(3) = robot->getBodyNodeCoMIsometry(DracoBodyNode::lFootCenter).translation();

    // Joint Position Task
    Eigen::VectorXd jpos_des = 0.95*robot->getQ().tail(Draco::n_adof);
    Eigen::VectorXd jvel_des(Draco::n_adof);  jvel_des.setZero();
    Eigen::VectorXd jacc_des(Draco::n_adof);  jacc_des.setZero();
    total_joint_task->updateTask(jpos_des, jvel_des, jacc_des);

    // myUtils::pretty_print(body_pos_des, std::cout, "body_pos_des");
    // myUtils::pretty_print(rfoot_pos_des, std::cout, "rfoot_pos_des");
    // myUtils::pretty_print(lfoot_pos_des, std::cout, "lfoot_pos_des");
    // myUtils::pretty_print(jpos_des, std::cout, "jpos_des");

    // Create the contacts
    ContactSpec* rfoot_front_contact = new PointContactSpec(robot, DracoBodyNode::rFootFront, 0.7);
    ContactSpec* rfoot_back_contact = new PointContactSpec(robot, DracoBodyNode::rFootBack, 0.7);
    ContactSpec* lfoot_front_contact = new PointContactSpec(robot, DracoBodyNode::lFootFront, 0.7);
    ContactSpec* lfoot_back_contact = new PointContactSpec(robot, DracoBodyNode::lFootBack, 0.7);

    contact_list.push_back(rfoot_front_contact);
    contact_list.push_back(rfoot_back_contact);
    contact_list.push_back(lfoot_front_contact);
    contact_list.push_back(lfoot_back_contact);

    // ((PointContactSpec*)rfoot_front_contact)->setMaxFz(100.0);

    rfoot_front_contact->updateContactSpec();
    rfoot_back_contact->updateContactSpec();
    lfoot_front_contact->updateContactSpec();
    lfoot_back_contact->updateContactSpec();

    // Update dynamics
    Eigen::MatrixXd A = robot->getMassMatrix();
    Eigen::MatrixXd Ainv = robot->getInvMassMatrix();
    Eigen::VectorXd grav = robot->getGravity();
    Eigen::MatrixXd coriolis = robot->getCoriolis();

    // myUtils::pretty_print(A, std::cout, "A");

    // Containers
    Eigen::VectorXd tau_cmd(robot->getNumDofs() - robot->getNumVirtualDofs()); // Torque Command output from IHBC
    Eigen::VectorXd qddot_cmd(robot->getNumDofs() - robot->getNumVirtualDofs()); // Joint Acceleration Command output from IHBC 

    int contact_dim_size = 0;
    for(int i = 0; i < contact_list.size(); i++){
        contact_dim_size += contact_list[i]->getDim();
    }
    ASSERT_EQ(contact_dim_size, 12);

    Eigen::VectorXd Fd(contact_dim_size); Fd.setZero();

    // Distribute reaction forces equally according to the mass of the robot
    double Fz_dist = (robot->getRobotMass()*9.81)/contact_list.size();
    int idx_offset = 0;
    for(int i = 0; i < contact_list.size(); i++){
        Fd[idx_offset + contact_list[i]->getFzIndex()] = Fz_dist;
        idx_offset += contact_list[i]->getDim();
    }

    // Initialize QP weights
    Eigen::VectorXd w_task_heirarchy(task_list.size());  // Vector of task priority weighs

    // Contact Tasks have weight 1.0
    // Pose Tasks have weight 1e-4
    // Joint Posture Tasks have weight 1e-6
    // w_task_heirarchy[0] = 1e-4; // Body
    // w_task_heirarchy[1] = 1.0; // rfoot
    // w_task_heirarchy[2] = 1.0; // lfoot
    // w_task_heirarchy[3] = 1e-6; // joint

    w_task_heirarchy[0] = 1.0; // rfoot
    w_task_heirarchy[1] = 1.0; // lfoot
    w_task_heirarchy[2] = 1e-6; // joint

    // When Fd is nonzero, we need to make the contact weight large if we want to trust the output of the 
    // MPC or planner
    // double w_contact_weight = 1e-10; // Contact Weight
    double w_contact_weight = 1.0; // Contact Weight
    // double w_contact_weight = 1.0/(robot->getRobotMass()*9.81);

    // Regularization terms should always be the lowest cost. 
    double lambda_qddot = 1e-16;
    double lambda_Fr = 1e-16;

    // Set QP weights
    ihwbc->setQPWeights(w_task_heirarchy, w_contact_weight);
    ihwbc->setRegularizationTerms(lambda_qddot, lambda_Fr);

    // Update and solve QP
    ihwbc->updateSetting(A, Ainv, coriolis, grav);
    ihwbc->solve(task_list, contact_list, Fd, tau_cmd, qddot_cmd);

    // QP dec variable results
    Eigen::VectorXd qddot_res;
    Eigen::VectorXd Fr_res;

    ihwbc->getQddotResult(qddot_res);
    ihwbc->getFrResult(Fr_res);

    double total_Fz = 0;
    idx_offset = 0;
    for(int i = 0; i < contact_list.size(); i++){
        total_Fz += Fr_res[idx_offset + contact_list[i]->getFzIndex()];
        idx_offset += contact_list[i]->getDim();
    }

    myUtils::pretty_print(Fd, std::cout, "Fd");
    myUtils::pretty_print(tau_cmd, std::cout, "tau_cmd");
    myUtils::pretty_print(qddot_cmd, std::cout, "qddot_cmd");
    myUtils::pretty_print(qddot_res, std::cout, "qddot_res");
    myUtils::pretty_print(Fr_res, std::cout, "Fr_res");
    std::cout << "robot mass = " << robot->getRobotMass() << std::endl;
    std::cout << "robot mass*grav = " << robot->getRobotMass()*9.81 << std::endl;
    std::cout << "total_Fz:" << total_Fz << std::endl;

    // Assert that computed reaction forces must be less than 1 Newton
    double diff_Fz = robot->getRobotMass()*9.81 - total_Fz;
    ASSERT_LE(std::fabs(diff_Fz), 1.0);

}

// Similar to above, the body task is optional if we know the desired reaction force.
// If we want to enforce both, we have to select whether we want to weigh the reaction force tracking
// or the body task tracking more. We use w_contact_weight to regulate the relationship
TEST(IHWBC, desired_contact_wrench_computation_no_body_task) {
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
    // myUtils::pretty_print(q, std::cout, "q");    

    // Task and Contact list
    std::vector<Task*> task_list;
    std::vector<ContactSpec*> contact_list;

    // Create Tasks

    // Body RxRyZ tasks or CoM xyz task or Linear Momentum Task
    // Body Rx Ry Rz
    // Foot location task.
    // Joint Position Task. It appears that it's important to have this task to ensure that uncontrolled qddot does not blow up
    Task* body_rpz_task_ = new BodyRxRyZTask(robot);
    Task* rfoot_center_rz_xyz_task = new FootRzXYZTask(robot, DracoBodyNode::rFootCenter);
    Task* lfoot_center_rz_xyz_task = new FootRzXYZTask(robot, DracoBodyNode::lFootCenter);
    Task* total_joint_task = new BasicTask(robot, BasicTaskType::JOINT, Draco::n_adof);

    // task_list.push_back(body_rpz_task_);
    task_list.push_back(rfoot_center_rz_xyz_task);
    task_list.push_back(lfoot_center_rz_xyz_task);
    task_list.push_back(total_joint_task);

    // Set Task Gains
    Eigen::VectorXd kp_body = 100*Eigen::VectorXd::Ones(3); 
    Eigen::VectorXd kd_body = 1.0*Eigen::VectorXd::Ones(3);
    body_rpz_task_->setGain(kp_body, kd_body);

    Eigen::VectorXd kp_foot = 100*Eigen::VectorXd::Ones(4); 
    Eigen::VectorXd kd_foot = 1.0*Eigen::VectorXd::Ones(4);
    rfoot_center_rz_xyz_task->setGain(kp_foot, kd_foot);
    lfoot_center_rz_xyz_task->setGain(kp_foot, kd_foot);

    Eigen::VectorXd kp_jp = Eigen::VectorXd::Ones(Draco::n_adof); 
    Eigen::VectorXd kd_jp = 0.1*Eigen::VectorXd::Ones(Draco::n_adof);
    total_joint_task->setGain(kp_jp, kd_jp);


    // Set the desired task values
    // Body Task
    Eigen::VectorXd body_pos_des(7); body_pos_des.setZero();
    Eigen::VectorXd body_vel_des(6); body_vel_des.setZero();    
    Eigen::VectorXd body_acc_des(6); body_acc_des.setZero();
    Eigen::Quaternion<double> body_des_quat(1, 0, 0, 0);

    body_pos_des[0] = body_des_quat.w();
    body_pos_des[1] = body_des_quat.x();
    body_pos_des[2] = body_des_quat.y();
    body_pos_des[3] = body_des_quat.z();

    body_pos_des[4] = q[0];
    body_pos_des[5] = q[1];
    body_pos_des[6] = q[2];
    
    body_rpz_task_->updateTask(body_pos_des, body_vel_des, body_acc_des);

    // Foot Task
    Eigen::VectorXd rfoot_pos_des(7); rfoot_pos_des.setZero();
    Eigen::VectorXd lfoot_pos_des(7); lfoot_pos_des.setZero();
    Eigen::VectorXd foot_vel_des(6); foot_vel_des.setZero();    
    Eigen::VectorXd foot_acc_des(6); foot_acc_des.setZero();

    Eigen::Quaternion<double> rfoot_ori_act(robot->getBodyNodeCoMIsometry(DracoBodyNode::rFootCenter).linear());
    Eigen::Quaternion<double> lfoot_ori_act(robot->getBodyNodeCoMIsometry(DracoBodyNode::lFootCenter).linear());

    rfoot_pos_des[0] = rfoot_ori_act.w();
    rfoot_pos_des[1] = rfoot_ori_act.x();
    rfoot_pos_des[2] = rfoot_ori_act.y();
    rfoot_pos_des[3] = rfoot_ori_act.z();
    rfoot_pos_des.tail(3) = robot->getBodyNodeCoMIsometry(DracoBodyNode::rFootCenter).translation();

    lfoot_pos_des[0] = lfoot_ori_act.w();
    lfoot_pos_des[1] = lfoot_ori_act.x();
    lfoot_pos_des[2] = lfoot_ori_act.y();
    lfoot_pos_des[3] = lfoot_ori_act.z();
    lfoot_pos_des.tail(3) = robot->getBodyNodeCoMIsometry(DracoBodyNode::lFootCenter).translation();

    // Joint Position Task
    Eigen::VectorXd jpos_des = 0.95*robot->getQ().tail(Draco::n_adof);
    Eigen::VectorXd jvel_des(Draco::n_adof);  jvel_des.setZero();
    Eigen::VectorXd jacc_des(Draco::n_adof);  jacc_des.setZero();
    total_joint_task->updateTask(jpos_des, jvel_des, jacc_des);

    // myUtils::pretty_print(body_pos_des, std::cout, "body_pos_des");
    // myUtils::pretty_print(rfoot_pos_des, std::cout, "rfoot_pos_des");
    // myUtils::pretty_print(lfoot_pos_des, std::cout, "lfoot_pos_des");
    // myUtils::pretty_print(jpos_des, std::cout, "jpos_des");

    // Create the contacts
    ContactSpec* rfoot_front_contact = new PointContactSpec(robot, DracoBodyNode::rFootFront, 0.7);
    ContactSpec* rfoot_back_contact = new PointContactSpec(robot, DracoBodyNode::rFootBack, 0.7);
    ContactSpec* lfoot_front_contact = new PointContactSpec(robot, DracoBodyNode::lFootFront, 0.7);
    ContactSpec* lfoot_back_contact = new PointContactSpec(robot, DracoBodyNode::lFootBack, 0.7);

    contact_list.push_back(rfoot_front_contact);
    contact_list.push_back(rfoot_back_contact);
    contact_list.push_back(lfoot_front_contact);
    contact_list.push_back(lfoot_back_contact);

    // ((PointContactSpec*)rfoot_front_contact)->setMaxFz(100.0);

    rfoot_front_contact->updateContactSpec();
    rfoot_back_contact->updateContactSpec();
    lfoot_front_contact->updateContactSpec();
    lfoot_back_contact->updateContactSpec();

    // Update dynamics
    Eigen::MatrixXd A = robot->getMassMatrix();
    Eigen::MatrixXd Ainv = robot->getInvMassMatrix();
    Eigen::VectorXd grav = robot->getGravity();
    Eigen::MatrixXd coriolis = robot->getCoriolis();

    // myUtils::pretty_print(A, std::cout, "A");

    // Containers
    Eigen::VectorXd tau_cmd(robot->getNumDofs() - robot->getNumVirtualDofs()); // Torque Command output from IHBC
    Eigen::VectorXd qddot_cmd(robot->getNumDofs() - robot->getNumVirtualDofs()); // Joint Acceleration Command output from IHBC 

    int contact_dim_size = 0;
    for(int i = 0; i < contact_list.size(); i++){
        contact_dim_size += contact_list[i]->getDim();
    }

    // Set A desired target contact wrench
    Eigen::VectorXd Fd(6); Fd.setZero();
    Fd[2] = (robot->getRobotMass()*9.81);

    // Initialize QP weights
    Eigen::VectorXd w_task_heirarchy(task_list.size());  // Vector of task priority weighs
    Eigen::VectorXd w_rf_contacts(contact_list.size());  // Vector of task priority weighs

    // Note that the distribution of contact weighting must sum to 1.0
    for(int i = 0; i < contact_list.size(); i++){
        w_rf_contacts[i] = 1.0/contact_list.size();
    }
    w_rf_contacts[0] = 0.35;
    w_rf_contacts[1] = 0.15;

    // Contact Tasks have weight 1.0
    // Pose Tasks have weight 1e-4
    // Joint Posture Tasks have weight 1e-6
    // w_task_heirarchy[0] = 1e-4; // Body
    // w_task_heirarchy[1] = 1.0; // rfoot
    // w_task_heirarchy[2] = 1.0; // lfoot
    // w_task_heirarchy[3] = 1e-6; // joint

    w_task_heirarchy[0] = 1.0; // rfoot
    w_task_heirarchy[1] = 1.0; // lfoot
    w_task_heirarchy[2] = 1e-6; // joint

    // When Fd is nonzero, we need to make the contact weight large if we want to trust the output of the 
    // MPC or planner
    // Any of these should work.
    // double w_contact_weight = 1e-10; // Contact Weight
    // double w_contact_weight = 1.0; // Contact Weight
    // double w_contact_weight = 1.0/(robot->getRobotMass()*9.81);
    double w_contact_weight = 1e-4/(robot->getRobotMass()*9.81);

    // Regularization terms should always be the lowest cost. 
    double lambda_qddot = 1e-16;
    double lambda_Fr = 1e-12;//1e-16;

    // Set QP weights
    ihwbc->setQPWeights(w_task_heirarchy, w_rf_contacts, w_contact_weight);
    ihwbc->setTargetWrenchMinimization(true);
    ihwbc->setRegularizationTerms(lambda_qddot, lambda_Fr);

    // Update and solve QP
    ihwbc->updateSetting(A, Ainv, coriolis, grav);
    ihwbc->solve(task_list, contact_list, Fd, tau_cmd, qddot_cmd);

    // QP dec variable results
    Eigen::VectorXd qddot_res;
    Eigen::VectorXd Fr_res;

    ihwbc->getQddotResult(qddot_res);
    ihwbc->getFrResult(Fr_res);

    double total_Fz = 0;
    int idx_offset = 0;
    for(int i = 0; i < contact_list.size(); i++){
        total_Fz += Fr_res[idx_offset + contact_list[i]->getFzIndex()];
        idx_offset += contact_list[i]->getDim();
    }

    myUtils::pretty_print(Fd, std::cout, "Fd");
    myUtils::pretty_print(tau_cmd, std::cout, "tau_cmd");
    myUtils::pretty_print(qddot_cmd, std::cout, "qddot_cmd");
    myUtils::pretty_print(qddot_res, std::cout, "qddot_res");
    myUtils::pretty_print(Fr_res, std::cout, "Fr_res");
    std::cout << "robot mass = " << robot->getRobotMass() << std::endl;
    std::cout << "robot mass*grav = " << robot->getRobotMass()*9.81 << std::endl;
    std::cout << "total_Fz:" << total_Fz << std::endl;

    // Assert that computed reaction forces must be less than 10 Newtons
    double diff_Fz = robot->getRobotMass()*9.81 - total_Fz;
    ASSERT_LE(std::fabs(diff_Fz), 10.0);

}