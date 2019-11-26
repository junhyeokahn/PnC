#include "gtest/gtest.h"

#include <Configuration.h>
#include <Eigen/Dense>
#include <Utils/IO/IOUtilities.hpp>
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