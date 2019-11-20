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
	// double robot_mass;
	// Eigen::MatrixXd I_body;

	// Eigen::VectorXd x0; 
	// Eigen::MatrixXd r_feet; // each column is a vector of contact locations [x,y,z]^T 

	// int horizon = 10;
	// double mpc_dt = 0.025;

	void setRobotMass(const double & robot_mass_in);
	void setRobotWorldInertia(const Eigen::MatrixXd & world_inertia_in);

	void setDt(const double & dt);	// MPC dt interval per horizon
	void setHorizon(const double & dt);	// MPC dt interval per horizon

	void setStartingState(const Eigen::VectorXd & x0_in);

	void assemble_vec_to_matrix(const int & n, const int & m, const Eigen::VectorXd & vec, Eigen::MatrixXd & mat_out);

	// for testing
	void simulate_toy_mpc();


private:
	Eigen::MatrixXd R_roll(const double & phi);
	Eigen::MatrixXd R_pitch(const double & theta);
	Eigen::MatrixXd R_yaw(const double & psi);

	Eigen::MatrixXd skew_sym_mat(const Eigen::MatrixXd & r);


	void integrate_robot_dynamics(const double & dt, const Eigen::VectorXd & x_current, const Eigen::MatrixXd & f_Mat, const Eigen::MatrixXd & r_feet,
	                              const double & mass, const Eigen::MatrixXd & I_body,
	                              Eigen::VectorXd & x_next);
	void cont_time_state_space(const double & mass, const Eigen::MatrixXd & I_body, const double & psi_in,
	                           const Eigen::MatrixXd & r_feet, 
	                           Eigen::MatrixXd & A, Eigen::MatrixXd & B);
	void discrete_time_state_space(const double & dt, const Eigen::MatrixXd & A, const Eigen::MatrixXd & B, Eigen::MatrixXd & Adt, Eigen::MatrixXd & Bdt);

	void qp_matrices(const int & horizon, const Eigen::MatrixXd & Adt, const Eigen::MatrixXd & Bdt, Eigen::MatrixXd & Aqp, Eigen::MatrixXd & Bqp);
	void get_force_constraints(const int & n_Fr, Eigen::MatrixXd & CMat, Eigen::VectorXd & cvec);
	void get_qp_constraints(const int & horizon, const Eigen::MatrixXd & CMat, const Eigen::VectorXd & cvec, Eigen::MatrixXd & Cqp, Eigen::VectorXd & cvec_qp);
	void get_qp_costs(const int & n, const int & m, const int & horizon, const Eigen::VectorXd & vecS_cost, const double & control_alpha, Eigen::MatrixXd & Sqp, Eigen::MatrixXd & Kqp);

	void solve_mpc_qp(const Eigen::MatrixXd & Aqp,  const Eigen::MatrixXd & Bqp, const Eigen::VectorXd & X_ref, 
					  const Eigen::VectorXd & x0,   const Eigen::MatrixXd & Sqp, const Eigen::MatrixXd & Kqp, 
					  const Eigen::MatrixXd & Cqp, const Eigen::VectorXd & cvec_qp,
					  Eigen::VectorXd & f_vec_out);
	
	void solve_mpc(const Eigen::VectorXd & x0, const Eigen::VectorXd & X_des, const Eigen::MatrixXd & r_feet,
	               const double & robot_mass, const Eigen::MatrixXd & I_body, 
	               const int & horizon, const double & mpc_dt, Eigen::VectorXd & f_vec_out);

	void get_desired_x(const int & horizon, Eigen::VectorXd & X_ref);


};