#include "gtest/gtest.h"

#include <Configuration.h>
#include <Eigen/Dense>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

// Robot, mpc and whole body controller
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/WBC/IHWBC/IHWBC.hpp>
#include <PnC/MPC/CMPC.hpp>


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

    q[2] = 0.893; //1.193;
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

TEST(MPC, toy_mpc){
  CMPC convex_mpc;

  Eigen::MatrixXd I_robot_body = 5.0*Eigen::MatrixXd::Identity(3,3); // Body Inertia matrix 

  // System Params
  double robot_mass = 50; //kg
  convex_mpc.setRobotMass(robot_mass); // (kilograms) 
  convex_mpc.setRobotInertia(I_robot_body);

  // MPC Params
  int horizon = 10;
  convex_mpc.setHorizon(horizon); // horizon timesteps 
  convex_mpc.setDt(0.025); // (seconds) per horizon
  convex_mpc.setMu(0.7); //  friction coefficient
  convex_mpc.setMaxFz(500); // (Newtons) maximum vertical reaction force per foot.

  // Set the cost vector
  Eigen::VectorXd cost_vec(13);
  // Vector cost indexing: <<  th1,  th2,  th3,  px,  py,  pz,   w1,  w2,   w3,   dpx,  dpy,  dpz,  g
  cost_vec << 0.25, 0.25, 10.0, 2.0, 2.0, 50.0, 0.0, 0.0, 0.30, 0.20, 0.2, 0.10, 0.0;
  convex_mpc.setCostVec(cost_vec);


  // Starting robot state
  // Current reduced state of the robot
  // x = [Theta, p, omega, pdot, g] \in \mathbf{R}^13
  Eigen::VectorXd x0(13); 

  double init_roll(0), init_pitch(0), init_yaw(0), 
         init_com_x(0), init_com_y(0), init_com_z(0.25), 
         init_roll_rate(0), init_pitch_rate(0), init_yaw_rate(0),
         init_com_x_rate(0), init_com_y_rate(0), init_com_z_rate(0);

  x0 = convex_mpc.getx0(init_roll, init_pitch, init_yaw,
                        init_com_x, init_com_y, init_com_z,
                        init_roll_rate, init_pitch_rate, init_yaw_rate,
                        init_com_x_rate, init_com_y_rate, init_com_z_rate);

  // Feet Configuration
  // Set Foot contact locations w.r.t world
  Eigen::MatrixXd r_feet(3, 4); // Each column is a reaction force in x,y,z 
  r_feet.setZero();
  double foot_length = 0.05; // 5cm distance between toe and heel
  double nominal_width = 0.1; // 10cm distance between left and right feet

  // 4 Contact Configuration
  // Right Foot Front 
  r_feet(0, 0) = foot_length/2.0;  // x
  r_feet(1, 0) = -nominal_width/2.0; // y
  // Right Foot Back 
  r_feet(0, 1) = -foot_length/2.0; // x
  r_feet(1, 1) = -nominal_width/2.0; //y
  // Left Foot Front 
  r_feet(0, 2) = foot_length/2.0;  // x
  r_feet(1, 2) = nominal_width/2.0; //y
  // Left Foot Back 
  r_feet(0, 3) = -foot_length/2.0; // x
  r_feet(1, 3) = nominal_width/2.0; //y

  int n_Fr = r_feet.cols(); // Number of contacts
  int n = 13;
  int m = 3*n_Fr;

  Eigen::VectorXd f_vec_out(m*horizon);
  Eigen::MatrixXd f_Mat(3, n_Fr); f_Mat.setZero();

  // Get constant desired reference
  Eigen::VectorXd x_des(n);

  x_des.setZero();
  x_des[0] = 0.0; //M_PI/8; //des roll orientation
  x_des[1] = 0.0 ;//-M_PI/8; //des pitch orientation
  x_des[2] = M_PI/12; // Yaw orientation

  x_des[3] = 0.0;//-0.1;//;0.75; // Set desired z height to be 0.75m from the ground
  x_des[5] = 0.75;//;0.75; // Set desired z height to be 0.75m from the ground

  Eigen::VectorXd X_des(n*horizon);
  convex_mpc.get_constant_desired_x(x_des, X_des);

  Eigen::VectorXd x_pred(n); // Container to hold the predicted state after 1 horizon timestep 

  // Solve the MPC
  convex_mpc.solve_mpc(x0, X_des, r_feet, x_pred, f_vec_out);
  // convex_mpc.print_f_vec(n_Fr, f_vec_out);

  double total_z_force = 0.0;
  f_Mat = convex_mpc.getMatComputedGroundForces();
  for (int j = 0; j < f_Mat.cols(); j++){
    std::cout << "  f" << j << ":" << f_Mat.col(j).transpose() << std::endl;
    total_z_force += f_Mat.col(j)[2];
  }

  std::cout << "total z force = " << total_z_force << std::endl;

  double robot_weight = robot_mass*9.81;
  ASSERT_GE(total_z_force, robot_weight);
}

TEST(MPC_IHWBC, zyx_rates_to_ang_vel){
    Eigen::Vector3d rpy; 
    Eigen::Vector3d rpy_rate; 
    rpy << 0, M_PI/4.0, M_PI/4.0;
    rpy_rate << 0.0, 0.0, 5.0;

    Eigen::Vector3d ang_vel = myUtils::EulerZYXRatestoAngVel(rpy[0], rpy[1], rpy[2],
                                                             rpy_rate[0], rpy_rate[1], rpy_rate[2]);

    ASSERT_DOUBLE_EQ(ang_vel[0], 0.0);
    ASSERT_DOUBLE_EQ(ang_vel[1], 0.0);
    ASSERT_DOUBLE_EQ(ang_vel[2], 5.0);

}

// Torque limit test
TEST(MPC_IHWBC, mpc_ihwbc_test) {
    RobotSystem* robot;
    robot = new RobotSystem(6, THIS_COM "RobotModel/Robot/Draco/DracoPnC_Dart.urdf");

    // Initialize and Update the robot
    Eigen::VectorXd q, qdot;
    getInitialConfiguration(robot, q, qdot);
    robot->updateSystem(q, qdot);
    // myUtils::pretty_print(q, std::cout, "q");    

    // Get the initial robot inertia
    Eigen::MatrixXd Ig_o = robot->getCentroidInertia();
    Eigen::MatrixXd I_robot_body = Ig_o.block(0,0,3,3);

    Eigen::VectorXd robot_com = robot->getCoMPosition();
    Eigen::VectorXd robot_com_rate = robot->getCoMVelocity();

    myUtils::pretty_print(robot_com, std::cout, "robot_com");
    myUtils::pretty_print(robot_com_rate, std::cout, "robot_com_rate");

    // Initialize Convex MPC
    CMPC convex_mpc;

    // System Params
    double robot_mass = robot->getRobotMass(); //kg
    convex_mpc.setRobotMass(robot_mass); // (kilograms) 
    convex_mpc.setRobotInertia(I_robot_body);

    // MPC Params
    int horizon = 10;
    convex_mpc.setHorizon(horizon); // horizon timesteps 
    convex_mpc.setDt(0.025); // (seconds) per horizon
    convex_mpc.setMu(0.7); //  friction coefficient
    convex_mpc.setMaxFz(500); // (Newtons) maximum vertical reaction force per foot.
    convex_mpc.rotateBodyInertia(false); // Assume we are always providing the world inertia

    // Set the cost vector
    Eigen::VectorXd cost_vec(13);
    // Vector cost indexing: <<  th1,  th2,  th3,  px,  py,  pz,   w1,  w2,   w3,   dpx,  dpy,  dpz,  g
    cost_vec << 0.25, 0.25, 10.0, 2.0, 2.0, 50.0, 0.0, 0.0, 0.30, 0.20, 0.2, 0.10, 0.0;
    double cost_factor = 8.0;
    cost_vec *= cost_factor;
    convex_mpc.setCostVec(cost_vec);

    // Starting robot state
    // Current reduced state of the robot
    // x = [Theta, p, omega, pdot, g] \in \mathbf{R}^13
    Eigen::VectorXd x0(13); 

    double init_roll(q[3]), init_pitch(q[4]), init_yaw(q[5]), 
         init_com_x(robot_com[0]), init_com_y(robot_com[1]), init_com_z(robot_com[2]), 
         init_roll_rate(qdot[3]), init_pitch_rate(qdot[4]), init_yaw_rate(qdot[5]),
         init_com_x_rate(robot_com_rate[0]), init_com_y_rate(robot_com_rate[1]), init_com_z_rate(robot_com_rate[2]);

    x0 = convex_mpc.getx0(init_roll, init_pitch, init_yaw,
                        init_com_x, init_com_y, init_com_z,
                        init_roll_rate, init_pitch_rate, init_yaw_rate,
                        init_com_x_rate, init_com_y_rate, init_com_z_rate);

    myUtils::pretty_print(x0, std::cout, "x0");

    // Feet Configuration
    // Set Foot contact locations w.r.t world
    Eigen::MatrixXd r_feet(3, 4); // Each column is a reaction force in x,y,z 
    r_feet.setZero();

   // Get the feet configuration
    Eigen::Vector3d feet_pos = robot->getBodyNodeCoMIsometry(DracoBodyNode::rFootFront).translation();
    r_feet.col(0) = robot->getBodyNodeCoMIsometry(DracoBodyNode::rFootFront).translation().transpose();
    r_feet.col(1) = robot->getBodyNodeCoMIsometry(DracoBodyNode::rFootBack).translation().transpose();
    r_feet.col(2) = robot->getBodyNodeCoMIsometry(DracoBodyNode::lFootFront).translation().transpose();
    r_feet.col(3) = robot->getBodyNodeCoMIsometry(DracoBodyNode::lFootBack).translation().transpose();

    myUtils::pretty_print(feet_pos, std::cout, "feet_pos");
    myUtils::pretty_print(r_feet, std::cout, "r_feet");

    int n_Fr = r_feet.cols(); // Number of contacts
    int n = 13;
    int m = 3*n_Fr;

    Eigen::VectorXd f_vec_out(m*horizon);
    Eigen::MatrixXd f_Mat(3, n_Fr); f_Mat.setZero();

    // Get constant desired reference
    Eigen::VectorXd x_des(n);

    // x = [Theta, p, omega, pdot, g] \in \mathbf{R}^13 
    x_des.setZero();
    x_des[0] = q[3]; //M_PI/8; //des roll orientation
    x_des[1] = q[4];//-M_PI/8; //des pitch orientation
    x_des[2] = q[5]; // Yaw orientation

    x_des[3] = robot_com[0] - 0.05; // Set desired com x pos
    x_des[4] = robot_com[1]; // Set desired com y pos 
    x_des[5] = robot_com[2] + 0.045;//;0.75; // Set desired z height to be the current + a small increment

    myUtils::pretty_print(x_des, std::cout, "x_des");

    Eigen::VectorXd X_des(n*horizon);
    convex_mpc.get_constant_desired_x(x_des, X_des);

    Eigen::VectorXd x_pred(n); // Container to hold the predicted state after 1 horizon timestep 

    // Solve the MPC
    convex_mpc.solve_mpc(x0, X_des, r_feet, x_pred, f_vec_out);

    myUtils::pretty_print(x_pred, std::cout, "x_pred");

    double total_z_force = 0.0;
    f_Mat = convex_mpc.getMatComputedGroundForces();
    std::cout << "Desired Forces on the feet" << std::endl;
    for (int j = 0; j < f_Mat.cols(); j++){
        std::cout << "  f" << j << ":" << f_Mat.col(j).transpose() << std::endl;
        total_z_force += f_Mat.col(j)[2];
    }

    Eigen::VectorXd Fd_current = convex_mpc.getComputedGroundForces();

    std::cout << "total z force = " << total_z_force << std::endl;
    myUtils::pretty_print(Fd_current , std::cout, "Fd_current");

    // Set desired com and body orientation from predicted state 
    double des_roll = x_pred[0];
    double des_pitch = x_pred[1];
    double des_yaw = x_pred[2];

    double des_pos_x = x_pred[3];
    double des_pos_y = x_pred[4];
    double des_pos_z = x_pred[5];

    double des_roll_rate = x_pred[6];
    double des_pitch_rate = x_pred[7];
    double des_yaw_rate = x_pred[8];

    double des_vel_x = x_pred[9];
    double des_vel_y = x_pred[10];
    double des_vel_z = x_pred[11];

    //  Initialize IHWBC
    //  Initialize actuated list
    std::vector<bool> act_list;
    act_list.resize(robot->getNumDofs(), true);
    for (int i(0); i < robot->getNumVirtualDofs(); ++i){
        act_list[i] = false;    
    } 
    IHWBC* ihwbc = new IHWBC(act_list);

    // Task and Contact list
    std::vector<Task*> task_list;
    std::vector<ContactSpec*> contact_list;

    // Create Tasks

    // Body RxRyZ tasks or CoM xyz task or Linear Momentum Task
    // Body Rx Ry Rz
    // Foot location task.
    // Joint Position Task. It appears that it's important to have this task to ensure that uncontrolled qddot does not blow up
    Task* body_rpz_task_ = new BodyRxRyZTask(robot);

    Task* com_task_ = new BasicTask(robot, BasicTaskType::COM, 3);
    Task* body_rpy_task_ = new BasicTask(robot, BasicTaskType::LINKORI, 3, DracoBodyNode::Torso);

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

    Eigen::VectorXd kp_com = 100*Eigen::VectorXd::Ones(3); 
    Eigen::VectorXd kd_com = 1.0*Eigen::VectorXd::Ones(3);
    com_task_->setGain(kp_com, kd_com);

    Eigen::VectorXd kp_body_rpy = 100*Eigen::VectorXd::Ones(3); 
    Eigen::VectorXd kd_body_rpy = 1.0*Eigen::VectorXd::Ones(3);
    body_rpy_task_->setGain(kp_body_rpy, kd_body_rpy);

    // COM Task
    Eigen::VectorXd com_pos_des(3); com_pos_des.setZero();
    Eigen::VectorXd com_vel_des(3); com_vel_des.setZero();    
    Eigen::VectorXd com_acc_des(3); com_acc_des.setZero();

    com_pos_des[0] = des_pos_x;
    com_pos_des[1] = des_pos_y;
    com_pos_des[2] = des_pos_z;

    com_vel_des[0] = des_vel_x;
    com_vel_des[1] = des_vel_y;
    com_vel_des[2] = des_vel_z;

    com_task_->updateTask(com_pos_des, com_vel_des, com_acc_des);
    myUtils::pretty_print(com_pos_des, std::cout, "com_pos_des");

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
    // double Fz_dist = (robot->getRobotMass()*9.81)/contact_list.size();
    int idx_offset = 0;
    // for(int i = 0; i < contact_list.size(); i++){
    //     Fd[idx_offset + contact_list[i]->getFzIndex()] = Fz_dist;
    //     idx_offset += contact_list[i]->getDim();
    // }

    // Set desired forces based on the MPC output 
    Fd = Fd_current;

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
    // Keeping it the same magnitude as the body task seems to have some benefits.
    // double w_contact_weight = 1e-4/(robot->getRobotMass()*9.81);
    double w_contact_weight = 1e-2/(robot->getRobotMass()*9.81);

    // Regularization terms should always be the lowest cost. 
    double lambda_qddot = 1e-16;
    double lambda_Fr = 1e-16;

    // Enable Torque Limits
    ihwbc->enableTorqueLimits(true);
    double tau_lim = 100.0;
    Eigen::VectorXd tau_min = -tau_lim*Eigen::VectorXd::Ones(Draco::n_adof);
    Eigen::VectorXd tau_max = tau_lim*Eigen::VectorXd::Ones(Draco::n_adof);
    ihwbc->setTorqueLimits(tau_min, tau_max);

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

    // Assert that the computed torques are less than the set limit
    for(int i = 0; i < tau_cmd.size(); i++){
       ASSERT_LE(std::fabs(tau_cmd[i]), (tau_lim+0.001));
    }

}

