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
#define MPC_TIME_ALL 

// We are following the MPC formulation from:
// Di Carlo, Jared, et al. "Dynamic locomotion in the mit cheetah 3 through convex model-predictive control." 
// 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2018.

class CMPC{
public: 
	CMPC();
	~CMPC();
	// Eigen::MatrixXd I_robot;

	// Eigen::VectorXd x0; 
	// Eigen::MatrixXd r_feet; // each column is a vector of contact locations [x,y,z]^T 

	int horizon;// mpc horizon (number of steps);
	double mpc_dt; // mpc time interval per horizon steo
	double mu; // coefficient of friction
	double fz_max; // maximum z reaction force for one force vector.

	double robot_mass; // kg mass of the robot
	bool rotate_inertia; // whether or not the inertia needs to be rotated to the world frame. 
						 // If the inertia is expressed in body frame this needs to be true.
						 // Otherwise if the inertia is already updated to be in the world frame, this can be set to false.

	// double mpc_dt = 0.025;

	void updateContacts(const Eigen::MatrixXd & r_in);

	void setRobotMass(const double robot_mass_in){ robot_mass = robot_mass_in; }
	void rotateBodyInertia(bool rotate_inertia_in){ rotate_inertia = rotate_inertia_in; }

	void setRobotInertia(const Eigen::MatrixXd & inertia_in);
	void setDt(const double mpc_dt_in){mpc_dt = mpc_dt_in;}	// MPC dt interval per horizon
	void setHorizon(const int & horizon_in){ horizon = horizon_in;}	// MPC horizon (number of steps)

	void setMu(const double mu_in){ mu = mu_in;} // Set the coefficient of friction
	void setMaxFz(const double fz_max_in){ fz_max = fz_max_in;} // Set the maximum z reaction force for one force vector


	void setStartingState(const Eigen::VectorXd & x0_in);

	void assemble_vec_to_matrix(const int & n, const int & m, const Eigen::VectorXd & vec, Eigen::MatrixXd & mat_out);

	// for testing
	void simulate_toy_mpc();


	void print_f_vec(int & n_Fr, const Eigen::VectorXd & f_vec_out);

	void get_constant_desired_x(const Eigen::VectorXd & x_des, Eigen::VectorXd & X_ref);

private:
	double gravity_acceleration;
  	Eigen::MatrixXd R_roll(const double & phi);
	Eigen::MatrixXd R_pitch(const double & theta);
	Eigen::MatrixXd R_yaw(const double & psi);

	Eigen::MatrixXd skew_sym_mat(const Eigen::MatrixXd & v);

	// I_robot is the inertia of the robot. If the values are in the body frame, then rotate_inertia must be set to true.
	// if I_robot is expressed in the world frame then rotate_inertia must be set to false.
	void integrate_robot_dynamics(const double & dt, const Eigen::VectorXd & x_current, const Eigen::MatrixXd & f_Mat, const Eigen::MatrixXd & r_feet,
	                              const Eigen::MatrixXd & I_robot,
	                              Eigen::VectorXd & x_next);
	void cont_time_state_space(const Eigen::MatrixXd & I_robot, const double & psi_in,
	                           const Eigen::MatrixXd & r_feet, 
	                           Eigen::MatrixXd & A, Eigen::MatrixXd & B);
	void discrete_time_state_space(const Eigen::MatrixXd & A, const Eigen::MatrixXd & B, Eigen::MatrixXd & Adt, Eigen::MatrixXd & Bdt);

	void qp_matrices(const Eigen::MatrixXd & Adt, const Eigen::MatrixXd & Bdt, Eigen::MatrixXd & Aqp, Eigen::MatrixXd & Bqp);
	void get_force_constraints(const int & n_Fr, Eigen::MatrixXd & CMat, Eigen::VectorXd & cvec);
	void get_qp_constraints(const Eigen::MatrixXd & CMat, const Eigen::VectorXd & cvec, Eigen::MatrixXd & Cqp, Eigen::VectorXd & cvec_qp);
	void get_qp_costs(const int & n, const int & m, const Eigen::VectorXd & vecS_cost, const double & control_alpha, Eigen::MatrixXd & Sqp, Eigen::MatrixXd & Kqp);

	void solve_mpc_qp(const Eigen::MatrixXd & Aqp,  const Eigen::MatrixXd & Bqp, const Eigen::VectorXd & X_ref, 
					  const Eigen::VectorXd & x0,   const Eigen::MatrixXd & Sqp, const Eigen::MatrixXd & Kqp, 
					  const Eigen::MatrixXd & Cqp, const Eigen::VectorXd & cvec_qp,
					  Eigen::VectorXd & f_vec_out);
	
	// x = [Theta, p, omega, pdot, g] \in \mathbf{R}^13	
	void solve_mpc(const Eigen::VectorXd & x0, const Eigen::VectorXd & X_des, const Eigen::MatrixXd & r_feet,
	               const Eigen::MatrixXd & I_robot, Eigen::VectorXd & x_pred, Eigen::VectorXd & f_vec_out);



};