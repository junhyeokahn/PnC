#ifndef H_CONVEX_MPC
#define H_CONVEX_MPC

// Standard
#include <math.h>
#include <iostream>
#include <stdio.h>

// Timer
#include <chrono>

// QP Solver
#include <ExternalSource/myOptimizer/Goldfarb/QuadProg++.hh>

// Eigen
#include <Eigen/Dense>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

// #define MPC_PRINT_ALL 
// #define MPC_TIME_ALL 

// We are following the MPC formulation from:
// Di Carlo, Jared, et al. "Dynamic locomotion in the mit cheetah 3 through convex model-predictive control." 
// 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2018.

class CMPC{
public: 
	CMPC();
	~CMPC();

	double robot_mass; // kg mass of the robot
	Eigen::MatrixXd I_robot; // Robot inertia

	int horizon;// mpc horizon (number of steps);
	double mpc_dt; // mpc time interval per horizon steo
	double mu; // coefficient of friction
	double fz_max; // maximum z reaction force for one force vector.
	double control_alpha_; // Regularization term on the controls

	bool rotate_inertia; // whether or not the inertia needs to be rotated to the world frame. 
						 // If the inertia is expressed in body frame this needs to be true.
						 // Otherwise if the inertia is already updated to be in the world frame, this can be set to false.

	Eigen::VectorXd latest_f_vec_out; // Container holding the the latest computed force output for control (not to be confused with the force at the end of the horizon)

  	// Vector cost for the MPC: <<  th1,  th2,  th3,  px,  py,  pz,   w1,  w2,   w3,   dpx,  dpy,  dpz,  g
	// last term is gravity and should always be 0,0
	// cost_vec << 0.25, 0.25, 10.0, 2.0, 2.0, 50.0, 0.0, 0.0, 0.30, 0.20, 0.2, 0.10, 0.0;
	Eigen::VectorXd cost_vec;  

	// Returns the state vector size of the MPC per horizon.
	int getStateVecDim(){return 13;}

	// Helper function which returns a 13-vector given the state of the robot. 
	// x = [Theta, p, omega, pdot, g] \in \mathbf{R}^13	
	Eigen::VectorXd getx0(const double roll, const double pitch, const double yaw,
						  const double com_x, const double com_y, const double com_z,
						  const double roll_rate, const double pitch_rate, const double yaw_rate,
						  const double com_x_rate, const double com_y_rate, const double com_z_rate);

	// Gets the latest computed ground forces for control
	Eigen::VectorXd getComputedGroundForces(){return latest_f_vec_out;}
	// Get computed ground forces in matrix form where the columns are the ground reaction forces
	// in the same order as r_feet. 
	Eigen::MatrixXd getMatComputedGroundForces();


	void setRobotMass(const double robot_mass_in){ robot_mass = robot_mass_in; }
	// Whether or not the inertia needs to be rotated to the world frame. 
	void rotateBodyInertia(bool rotate_inertia_in){ rotate_inertia = rotate_inertia_in; }

	void setRobotInertia(const Eigen::MatrixXd & inertia_in){ I_robot = inertia_in; }
	void setDt(const double mpc_dt_in){mpc_dt = mpc_dt_in;}	// MPC dt interval per horizon
	void setHorizon(const int & horizon_in){ horizon = horizon_in;}	// MPC horizon (number of steps)

	void setMu(const double mu_in){ mu = mu_in;} // Set the coefficient of friction
	void setMaxFz(const double fz_max_in){ fz_max = fz_max_in;} // Set the maximum z reaction force for one force vector

  	// Vector cost for the MPC: <<  th1,  th2,  th3,  px,  py,  pz,   w1,  w2,   w3,   dpx,  dpy,  dpz,  g
	void setCostVec(const Eigen::VectorXd & cost_vec_in); // Sets the cost vector.

	void setControlAlpha(const double control_alpha_in){control_alpha_ = control_alpha_in;}

	// Human readable prints out of f_vec_out. 
	void print_f_vec(int & n_Fr, const Eigen::VectorXd & f_vec_out);

	// Helper function which transforms a constant x_des to a constant trajectory reference
	// Input: x_des \in \mathbf{R}^{13}
	// Ouput: X_ref \in \mathbf{R}^{13*horizon}
	void get_constant_desired_x(const Eigen::VectorXd & x_des, Eigen::VectorXd & X_ref);

	// The main solve MPC routine 
	// All the values are in world frame
	// Inputs:
	// 		x0 = [Theta, p, omega, pdot, g] \in \mathbf{R}^{13}    (Starting state of the system) 		
	// 		X_des \in \mathbf{R}^{13*horizon} 				     (Reference or desired state evolution) 
	// 		r_feet \in \mathbf{R}^{3 x number_of_point_contacts} (Matrix of point contact locations where each column is the xyz location of the contact)
	// Outputs:
	//		x_pred \in \mathbf{R}^{13} - The predicted state after a time interval of mpc_dt has dimension  
	//		f_vec_out \mathbf{R}^{3 x number_of_point_contacts} The xyz forces needed to be exerted at the contact points to track X_des.

	void solve_mpc(const Eigen::VectorXd & x0, const Eigen::VectorXd & X_des, const Eigen::MatrixXd & r_feet,
	               Eigen::VectorXd & x_pred, Eigen::VectorXd & f_vec_out);

	// For testing
	void simulate_toy_mpc();


private:
	Eigen::MatrixXd r_feet_; // Store the value of r_feet locally.

	double gravity_acceleration;
  	Eigen::MatrixXd R_roll(const double & phi);
	Eigen::MatrixXd R_pitch(const double & theta);
	Eigen::MatrixXd R_yaw(const double & psi);
	Eigen::MatrixXd skew_sym_mat(const Eigen::VectorXd & v);

	void assemble_vec_to_matrix(const int & n, const int & m, const Eigen::VectorXd & vec, Eigen::MatrixXd & mat_out);

	void integrate_robot_dynamics(const double & dt, const Eigen::VectorXd & x_current, const Eigen::MatrixXd & f_Mat, const Eigen::MatrixXd & r_feet,
	                              Eigen::VectorXd & x_next);
	void cont_time_state_space(const Eigen::VectorXd & x_current,
	                           const Eigen::MatrixXd & r_feet, 
	                           Eigen::MatrixXd & A, Eigen::MatrixXd & B);
	void discrete_time_state_space(const Eigen::MatrixXd & A, const Eigen::MatrixXd & B, Eigen::MatrixXd & Adt, Eigen::MatrixXd & Bdt);

	void qp_matrices(const Eigen::MatrixXd & Adt, const Eigen::MatrixXd & Bdt, Eigen::MatrixXd & Aqp, Eigen::MatrixXd & Bqp);
	void get_force_constraints(const int & n_Fr, Eigen::MatrixXd & CMat, Eigen::VectorXd & cvec);
	void get_qp_constraints(const Eigen::MatrixXd & CMat, const Eigen::VectorXd & cvec, Eigen::MatrixXd & Cqp, Eigen::VectorXd & cvec_qp);
	void get_qp_costs(const int & n, const int & m, const Eigen::VectorXd & vecS_cost, const double & control_alpha, Eigen::MatrixXd & Sqp, Eigen::MatrixXd & Kqp);

	// Converts location of the feet expressed in world frame to the CoM frame. We assume the CoM orientation frame is always aligned with world.
	void convert_r_feet_to_com_frame(const Eigen::VectorXd & p_com, const Eigen::MatrixXd & r_feet, Eigen::MatrixXd & r_feet_com);

	void solve_mpc_qp(const Eigen::MatrixXd & Aqp,  const Eigen::MatrixXd & Bqp, const Eigen::VectorXd & X_ref, 
					  const Eigen::VectorXd & x0,   const Eigen::MatrixXd & Sqp, const Eigen::MatrixXd & Kqp, 
					  const Eigen::MatrixXd & Cqp, const Eigen::VectorXd & cvec_qp,
					  Eigen::VectorXd & f_vec_out);


};

#endif