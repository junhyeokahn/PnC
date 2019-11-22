#include <PnC/MPC/CMPC.hpp>

// Constructor 
CMPC::CMPC(){
  // Set Defaults
  robot_mass = 50; // kg mass of the robot
  I_robot = Eigen::MatrixXd::Identity(3,3); // initialize as the inertia matrix w.r.t to the body. 
  rotate_inertia = true; // whether or not the inertia (expressed in the body frame) needs to be rotated to the world frame
  // If I_robot is expressed in world frame and is regularly updated to be always in world, then rotate_inertia should be false

  horizon = 20; // Default horizon to 10 steps
  mpc_dt = 0.025; // MPC time interval per horizon steo
  mu = 0.9; // coefficient of friction
  fz_max = 500; // maximum z reaction force for one force vector.

  // The latest force output computed by the MPC as the control input for the robot.
  latest_f_vec_out = Eigen::VectorXd(0);

  // DO NOT CHANGE
  gravity_acceleration = -9.81;
}

// Destructor
CMPC::~CMPC(){}

// Helper function which returns a 13-vector given the state of the robot. 
// x = [Theta, p, omega, pdot, g] \in \mathbf{R}^13 
Eigen::VectorXd CMPC::getx0(const double roll, const double pitch, const double yaw,
            const double com_x, const double com_y, const double com_z,
            const double roll_rate, const double pitch_rate, const double yaw_rate,
            const double com_x_rate, const double com_y_rate, const double com_z_rate){
  Eigen::VectorXd x0(13);
  x0.setZero();
  x0 << roll, pitch, yaw, // Theta \in R^3 
        com_x, com_y, com_z,  // p \in R^3
        roll_rate, pitch_rate, yaw_rate, // omega \in R^3
        com_x_rate, com_y_rate, com_z_rate, // pdot \in R^3
        gravity_acceleration; // g \in R
  return x0;
}

Eigen::MatrixXd CMPC::R_roll(const double & phi){
  Eigen::MatrixXd Rx(3,3);
  Rx.setZero();
  Rx <<     1.0,    0.0, 0.0,
            0.0,   cos(phi), -sin(phi),
            0.0,   sin(phi),  cos(phi);
  return Rx; 
}

Eigen::MatrixXd CMPC::R_pitch(const double & theta){
  Eigen::MatrixXd Ry(3,3);
  Ry.setZero();
  Ry << cos(theta), 0.0, sin(theta),
            0.0,    1.0, 0.0,
       -sin(theta), 0.0, cos(theta);
  return Ry; 
}

Eigen::MatrixXd CMPC::R_yaw(const double & psi){
	Eigen::MatrixXd Rz(3,3);
	Rz.setZero();
	Rz << cos(psi), -sin(psi),  0.0,
		    sin(psi),  cos(psi),  0.0,
		    0.0,         0.0,     1.0;
	return Rz; 
}

Eigen::MatrixXd CMPC::skew_sym_mat(const Eigen::VectorXd & v){
  Eigen::MatrixXd ssm(3,3);
  ssm << 0.f, -v[2], v[1],
    v[2], 0.f, -v[0],
    -v[1], v[0], 0.f;
  return ssm;
}

void CMPC::convert_r_feet_to_com_frame(const Eigen::VectorXd & p_com, const Eigen::MatrixXd & r_feet, Eigen::MatrixXd & r_feet_com){
  r_feet_com = r_feet;
  for(int i = 0; i < r_feet.cols(); i++){
    r_feet_com.col(i) = r_feet.col(i) - p_com;
  }
  // std::cout << "p_com" << std::endl;
  // std::cout << p_com << std::endl;  

  // std::cout << "r_feet:" << std::endl;
  // std::cout << r_feet << std::endl; 

  // std::cout << "r_feet_com:" << std::endl;
  // std::cout << r_feet_com << std::endl;   
}

// Transform input vector to nxm matrix. Vector must have dimension n*m.
void CMPC::assemble_vec_to_matrix(const int & n, const int & m, const Eigen::VectorXd & vec, Eigen::MatrixXd & mat_out){
  mat_out = Eigen::MatrixXd::Zero(n, m);
  for(int j = 0; j < m; j++){
    mat_out.col(j) = vec.segment(j*n, n);
  }
  // std::cout << "mat_out:" << std::endl;
  // std::cout << mat_out << std::endl;
}


// Given dt, current x state, f_Mat, and r_feet. Find the next state, x1 
void CMPC::integrate_robot_dynamics(const double & dt, const Eigen::VectorXd & x_current, const Eigen::MatrixXd & f_Mat, const Eigen::MatrixXd & r_feet,
                                    Eigen::VectorXd & x_next){
  // x = [Theta, p, omega, pdot, g] \in \mathbf{R}^13
  double roll = x_current[0]; double pitch = x_current[1]; double yaw = x_current[2];
  // std::cout << "x_current:" << x_current.transpose() << std::endl;

  Eigen::VectorXd theta_prior = x_current.head(3);
  Eigen::VectorXd p_prior = x_current.segment(3,3);
  Eigen::VectorXd omega_prior = x_current.segment(6, 3);
  Eigen::VectorXd pdot_prior = x_current.segment(9,3);

  Eigen::MatrixXd Rx = R_roll(roll);
  Eigen::MatrixXd Ry = R_pitch(pitch);
  Eigen::MatrixXd Rz = R_yaw(yaw);

  Eigen::MatrixXd R_zyx = Rz*Ry*Rx;

  Eigen::MatrixXd I_world;
  if (rotate_inertia){
    I_world = R_zyx * I_robot * R_zyx.transpose(); // Inertia matrix aligned to CoM frame.
  }else{
    I_world = I_robot;
  }

  // Convert r_feet to be expressed in the CoM frame
  Eigen::MatrixXd r_feet_com = r_feet;
  convert_r_feet_to_com_frame(p_prior, r_feet, r_feet_com);

  // CoM acceleration, velocity, and position
  Eigen::VectorXd pddot(3); // pddot = 1/m sum(f_i) - g
  Eigen::VectorXd pdot(3);  // pdot = pddot*dt + pdot prior;
  Eigen::VectorXd p(3);     // p = 0.5*pddot*dt*dt + pdot_prior*dt + p_prior;

  Eigen::VectorXd grav_vec(3); grav_vec.setZero(); grav_vec[2] = gravity_acceleration;
  pddot.setZero();
  pddot += grav_vec;
  for(int i = 0; i < f_Mat.cols(); i++){
    pddot += ((1.0/robot_mass)*f_Mat.col(i));
  }
  pdot = pddot*dt + pdot_prior;   // CoM velocity
  p = 0.5*pddot*dt*dt + pdot_prior*dt + p_prior;   // CoM position

  // Angular Acceleration, velocity, and new orientation
  Eigen::VectorXd omega_dot(3); // omega_dot = I_inv * (sum(skew(r_i)*f_i) -skew(omega_prior)*I*omega_prior
  Eigen::VectorXd omega(3);     // omega = omega_dot*dt + omega_prior;
  Eigen::VectorXd theta;    // theta = Omega_Rate*(0.5*omega_dot*dt + omega_prior)*dt + theta_prior

  // ZYX from ZYX rates integration
  Eigen::MatrixXd omega_to_zyx_rate(3,3);
  omega_to_zyx_rate <<  cos(yaw)/cos(pitch),            sin(yaw)/cos(pitch),         0,
                            -sin(yaw),                        cos(yaw),              0,
                  cos(yaw)*sin(pitch)/cos(pitch),  sin(yaw)*sin(pitch)/cos(pitch),   1;

  Eigen::VectorXd moments(3); moments.setZero();
  for(int i = 0; i < f_Mat.cols(); i++){
    moments += skew_sym_mat(r_feet_com.col(i))*f_Mat.col(i);
  }
  omega_dot = I_world.inverse()*(moments - skew_sym_mat(omega_prior)*(I_world*omega_prior)) ;
  omega = omega_dot*dt + omega_prior;
  theta = omega_to_zyx_rate*(0.5*omega_dot*dt + omega_prior)*dt + theta_prior;

  // std::cout << "pddot:" << pddot.transpose() << std::endl;
  // std::cout << "pdot:" << pdot.transpose() << std::endl;
  // std::cout << "p_prior:" << p_prior.transpose() << std::endl;
  // std::cout << "p_post:" <<  p.transpose() << std::endl;

  // std::cout << std::endl;
  // std::cout << "omega_dot:" << omega_dot.transpose() << std::endl;
  // std::cout << "omega:" << omega.transpose() << std::endl;

  // std::cout << "omega_to_zyx_rate" << std::endl;
  // std::cout << omega_to_zyx_rate << std::endl;

  // std::cout << "theta_prior:" << theta_prior.transpose() << std::endl;
  // std::cout << "theta_post:" << theta.transpose() << std::endl;

  // Set new x_next;
  x_next.head(3) =  theta;
  x_next.segment(3, 3) = p;
  x_next.segment(6, 3) = omega;
  x_next.segment(9, 3) = pdot;
  x_next[12] = gravity_acceleration;

  // std::cout << "x_next:" << x_next.transpose() << std::endl;

}


// Inputs: m, I, current yaw, and footstep locations.
// Output: A, B matrix to be used for zero-order hold MPC
void CMPC::cont_time_state_space(const Eigen::VectorXd & x_current, const Eigen::MatrixXd & r_feet, 
                           Eigen::MatrixXd & A, Eigen::MatrixXd & B){
	// Get System State: CoM and Current Robot yaw
  Eigen::VectorXd p_com = x_current.segment(3,3);
  double psi_yaw = x_current[2];
	Eigen::MatrixXd Rz = R_yaw(psi_yaw);

  Eigen::MatrixXd I_world;
  if (rotate_inertia){
    I_world = Rz * I_robot * Rz.transpose(); // Inertia matrix aligned to CoM frame.
  }else{
    I_world = I_robot;
  }

	Eigen::MatrixXd I_inv = I_world.inverse(); // Inverse


  // Convert r_feet to be expressed in the CoM frame
  Eigen::MatrixXd r_feet_com = r_feet;
  convert_r_feet_to_com_frame(p_com, r_feet, r_feet_com);

	// Number of reaction forces equal to number of feet contacts
	int n_Fr = r_feet_com.cols();

	// x = [Theta, p, omega, pdot, g] \in \mathbf{R}^13
	// There is an additional gravity state.
	// Populate A matrix
	A = Eigen::MatrixXd::Zero(13,13);
	A.block(0, 6, 3, 3) = Rz;
	A.block(3, 9, 3, 3) = Eigen::MatrixXd::Identity(3,3);
	A(11, 12) = 1.0; // Gravity vector

	// Populate B matrix
	B = Eigen::MatrixXd::Zero(13, 3*n_Fr);
	Eigen::MatrixXd eye3 = Eigen::MatrixXd::Identity(3,3);	
	for(int i = 0; i < n_Fr; i++){
		B.block(6, i*3, 3, 3) = I_inv*skew_sym_mat(r_feet_com.col(i)); 
		B.block(9, i*3, 3, 3) = eye3*1.0/robot_mass;
	}

  #ifdef MPC_PRINT_ALL
  	std::cout << "Rz:" << std::endl;
  	std::cout << Rz << std::endl;

  	std::cout << "I_robot:" << std::endl;
  	std::cout << I_robot << std::endl;

  	std::cout << "I_world:" << std::endl;
  	std::cout << I_world << std::endl;

  	std::cout << "I_inv:" << std::endl;
  	std::cout << I_inv << std::endl;

  	std::cout << "r_feet_com:" << std::endl;
  	std::cout << r_feet_com << std::endl;

  	std::cout << "A:" << std::endl;
  	std::cout << A << std::endl;

  	std::cout << "B:" << std::endl;
  	std::cout << B << std::endl;

  	std::cout << "A.exp()" << std::endl;
  	std::cout << A.exp() << std::endl;
  #endif
}


void CMPC::discrete_time_state_space(const Eigen::MatrixXd & A, const Eigen::MatrixXd & B, Eigen::MatrixXd & Adt, Eigen::MatrixXd & Bdt){
	// p.215 Raymond DeCarlo: Linear Systems: A State Variable Approach with Numerical Implementation, Prentice Hall, NJ, 1989
	int n = A.cols() + B.cols();
	Eigen::MatrixXd AB(n,n);
	Eigen::MatrixXd expAB(n,n);
	AB.setZero();
	AB.block(0, 0, A.cols(), A.cols()) = A;
	AB.block(0, A.cols(), A.cols(), B.cols()) = B;

	expAB = (AB*mpc_dt).exp();

	Adt = expAB.block(0, 0, A.cols(), A.cols());
	Bdt = expAB.block(0, A.cols(), A.cols(), B.cols());

  #ifdef MPC_PRINT_ALL
  	std::cout << "Adt" << std::endl;
  	std::cout << Adt << std::endl;

  	std::cout << "Bdt" << std::endl;
  	std::cout << Bdt*mpc_dt << std::endl;
  #endif
}

// Zero order hold qp matrices
void CMPC::qp_matrices(const Eigen::MatrixXd & Adt, const Eigen::MatrixXd & Bdt, Eigen::MatrixXd & Aqp, Eigen::MatrixXd & Bqp){
	if (horizon < 1){
		std::cerr << "Error! MPC horizon must be at least 1 timestep long." << std::endl;
	}

	int n = Adt.cols();
	int m = Bdt.cols();
	std::vector<Eigen::MatrixXd> powerMats;
	for(int i = 0; i < horizon + 1; i++){
		powerMats.push_back(Eigen::MatrixXd::Identity(n, n));
	}

	// Construct power matrices foe Adt
	for(int i = 1; i < horizon + 1; i++){
		powerMats[i] = powerMats[i-1]*Adt;
	}

	// Construct Aqp and Bqp matrices
	Aqp.setZero();
	Bqp.setZero();

	for(int i = 0; i < horizon; i++){
		Aqp.block(i*n, 0, n, n) = powerMats[i+1]; // Adt.pow(i+1);
		for(int j = 0; j < horizon; j++){
			if (i >= j){
				int a_num = i-j;
				Bqp.block(i*n, j*m, n, m) = powerMats[a_num]*Bdt; // Adt.pow(a_num)*Bdt;				
			}
		}
	}

	// Print power matrices for Adt
	// for(int i = 0; i < horizon; i++){
	// 	std::cout << "powerMats[" << i << "]" << std::endl;
	// 	std::cout << powerMats[i] << std::endl;
	// }

  #ifdef MPC_PRINT_ALL
  	std::cout << "Aqp" << std::endl;
  	std::cout << Aqp << std::endl;

  	std::cout << "Bqp" << std::endl;
  	std::cout << Bqp << std::endl;
  #endif
}


void CMPC::get_force_constraints(const int & n_Fr, Eigen::MatrixXd & CMat, Eigen::VectorXd & cvec){
	// Unilateral constraints on the force vector
	// CMat * u + cvec >= 0
	Eigen::MatrixXd cfmat(6,3); // the constraint matrix for a single force vector	
	Eigen::VectorXd cfvec(6); // the constraint vector for a single force vector	

	cfmat.setZero();
	cfvec.setZero();

	// -mu*fz <= fx <= mu*fz
	cfmat(0,0) = -1; cfmat(0,2) = mu; 
	cfmat(1,0) = 1; cfmat(1,2) = mu;

	// -mu*fz <= fy <= mu*fz
	cfmat(2,1) = -1; cfmat(2,2) = mu;
	cfmat(3,1) = 1; cfmat(3,2) = mu;

	// fz_min = 0.0 <= fz <= fz_max
	cfmat(4,2) = -1.0; cfvec[4] = fz_max;// -fz + fz_max >= 0
	cfmat(5,2) = 1.0;

  #ifdef MPC_PRINT_ALL

  	std::cout << "cfmat " << std::endl;
  	std::cout << cfmat << std::endl;

  	std::cout << "cfvec " << std::endl;
  	std::cout << cfvec << std::endl;
  #endif

	// Populate constraint matrix
	CMat.setZero(); cvec.setZero();
	for(int i = 0; i < n_Fr; i++){
		CMat.block(6*i, 3*i, 6, 3) = cfmat;
		cvec.segment(6*i, 6) = cfvec;
	}

  #ifdef MPC_PRINT_ALL
  	std::cout << "CMat " << std::endl;
  	std::cout << CMat << std::endl;

  	std::cout << "cvec " << std::endl;
  	std::cout << cvec << std::endl;
  #endif
}

void CMPC::get_qp_constraints(const Eigen::MatrixXd & CMat, const Eigen::VectorXd & cvec, Eigen::MatrixXd & Cqp, Eigen::VectorXd & cvec_qp){
	Cqp.setZero();
	cvec_qp.setZero();

	int num_rows = CMat.rows();
	int num_cols = CMat.cols();

	// Set the force constraints over the horizon
  // To DO: add 0 upperbound when reaction force should be 0.0 depending on the gate cycle. 
	for(int i = 0; i < horizon; i++){
		Cqp.block(i*num_rows, i*num_cols, num_rows, num_cols) = CMat;
		cvec_qp.segment(i*num_rows, num_rows) = cvec;
	}

  #ifdef MPC_PRINT_ALL
  	std::cout << "Cqp" << std::endl;
  	std::cout << Cqp.rows() << "x" << Cqp.cols() << std::endl;
  	std::cout << Cqp << std::endl;

  	std::cout << "cvec_qp" << std::endl;
  	std::cout << cvec_qp << std::endl;
  #endif

}

void CMPC::get_qp_costs(const int & n, const int & m, const Eigen::VectorXd & vecS_cost, const double & control_alpha, Eigen::MatrixXd & Sqp, Eigen::MatrixXd & Kqp){
	Eigen::MatrixXd S_cost = vecS_cost.asDiagonal();
	Sqp.setZero();
	for (int i = 0; i < horizon; i++){
		Sqp.block(n*i, n*i, n, n) = S_cost;
	}

	Kqp = control_alpha*Eigen::MatrixXd::Identity(m*horizon, m*horizon);

  #ifdef MPC_PRINT_ALL
    std::cout << "S_cost" << std::endl;
    std::cout << S_cost << std::endl; 

  	std::cout << "Sqp" << std::endl;
  	std::cout << Sqp << std::endl;

  	std::cout << "Kqp" << std::endl;
  	std::cout << Kqp << std::endl;
  #endif
}

void CMPC::solve_mpc_qp(const Eigen::MatrixXd & Aqp,  const Eigen::MatrixXd & Bqp, const Eigen::VectorXd & X_ref, 
				  const Eigen::VectorXd & x0,   const Eigen::MatrixXd & Sqp, const Eigen::MatrixXd & Kqp, 
				  const Eigen::MatrixXd & Cqp, const Eigen::VectorXd & cvec_qp,
				  Eigen::VectorXd & f_vec_out){

  #ifdef MPC_TIME_ALL
    std::chrono::high_resolution_clock::time_point t_qp_prep_start;
    std::chrono::high_resolution_clock::time_point t_qp_prep_end;
    double dur_qp_prep = 0.0;

    std::chrono::high_resolution_clock::time_point t_qp_mult_start;
    std::chrono::high_resolution_clock::time_point t_qp_mult_end;
    double dur_qp_mult = 0.0;

    t_qp_prep_start = std::chrono::high_resolution_clock::now();
    t_qp_mult_start = std::chrono::high_resolution_clock::now();
  #endif

  Eigen::MatrixXd H(Bqp.cols(), Bqp.cols());
  Eigen::VectorXd g_qp(Bqp.cols());

  // For some reason g_qp evaluates slow unless we do these operations first
  // std::cout << "x0" << x0.transpose() << std::endl;
  Eigen::MatrixXd BqpT = Bqp.transpose();
  Eigen::MatrixXd m1; m1.noalias() = BqpT*Sqp;
  Eigen::MatrixXd m2; m2.noalias() = 2.0*m1;
  Eigen::VectorXd v1 = (Aqp*x0).eval() - X_ref; 

  H = 2.0*(m1*Bqp + Kqp);
  g_qp.noalias() = m1*v1;	

  #ifdef MPC_TIME_ALL
    t_qp_mult_end = std::chrono::high_resolution_clock::now();
  #endif

  // std::cout << "H" << std::endl;
  // std::cout << H << std::endl;

  // std::cout << "g" << std::endl;
  // std::cout << g.transpose() << std::endl;

  // To Do: Add caching. Use previous solution as initial guess 

  // Quadprog matrices vectors
  GolDIdnani::GMatr<double> G, CE, CI;
  GolDIdnani::GVect<double> g0, ce0, ci0, x;  
  int n_quadprog_ = 1;
  int p_quadprog_ = 0;
  int m_quadprog_ = 0;

  // Quadprog size setup
  n_quadprog_ = Bqp.cols(); // number of decision variables
  m_quadprog_ = 0;  // number of equality constraints
  p_quadprog_ = cvec_qp.size(); // number of inequality constraints (unilateral force constraints). 

  // std::cout << "n_quadprog_" << n_quadprog_ << std::endl;
  // std::cout << "p_quadprog_" << p_quadprog_ << std::endl;

  f_vec_out.resize(n_quadprog_);

  x.resize(n_quadprog_);   // Decision Variables
  // Objective
  G.resize(n_quadprog_, n_quadprog_);
  g0.resize(n_quadprog_);
  // Equality Constraints
  CE.resize(n_quadprog_, m_quadprog_);
  ce0.resize(m_quadprog_);
  // Inequality Constraints
  CI.resize(n_quadprog_, p_quadprog_);  
  ci0.resize(p_quadprog_);

  // Solve the following QP:
  // min (1/2)*U^T*H*U + U^T*g
  // st. C*U <= cvec_qp
  // where U is the decision variable (vector of forces)

  // Populate G Cost Matrix
  // G = P
  for(int i = 0; i < n_quadprog_; i++){
    for(int j = 0; j < n_quadprog_; j++){
      G[i][j] = H(i,j);
    }
  }

  //Populate g0 cost vector
  for(int i = 0; i < n_quadprog_; i++){
    g0[i] = g_qp[i];
  }

  // No equality constraints. So CE and ce0 are untouched.

  // Populate Inequality Constraint Matrix
  for(int i = 0; i < p_quadprog_; i++){
    for(int j = 0; j < n_quadprog_; j++){
      CI[j][i] = Cqp(i,j);
    }
  }

  // Populate Inequality Constraint Vector
  // ci0 = h_
  for(int i = 0; i < p_quadprog_; i++){
    ci0[i] = cvec_qp[i];
  }

  #ifdef MPC_TIME_ALL

  t_qp_prep_end = std::chrono::high_resolution_clock::now();
  dur_qp_mult = std::chrono::duration_cast< std::chrono::duration<double> >(t_qp_mult_end - t_qp_mult_start).count();
  dur_qp_prep = std::chrono::duration_cast< std::chrono::duration<double> >(t_qp_prep_end - t_qp_prep_start).count();

  std::cout << "QP mult took " << dur_qp_mult << " seconds." << std::endl; 
  std::cout << "QP mult rate " << (1.0/dur_qp_mult) << " Hz" << std::endl; 
  std::cout << "QP prep took " << dur_qp_prep << " seconds." << std::endl; 
  std::cout << "QP prep rate " << (1.0/dur_qp_prep) << " Hz" << std::endl; 

  // Time the QP solve
  std::chrono::high_resolution_clock::time_point t_qp_solve_start;
  std::chrono::high_resolution_clock::time_point t_qp_solve_end;
  double dur_qp_solve = 0.0;

  // Start
  t_qp_solve_start = std::chrono::high_resolution_clock::now();
  #endif

  double qp_result = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);

  #ifdef MPC_TIME_ALL
    // End
    t_qp_solve_end = std::chrono::high_resolution_clock::now();
    dur_qp_solve = std::chrono::duration_cast< std::chrono::duration<double> >(t_qp_solve_end - t_qp_solve_start).count();
  #endif

  // Populate qd result from QP answer
  for(int i = 0; i < n_quadprog_; i++){
    f_vec_out[i] = x[i];
  }
  // std::cout << "qp_result = " << qp_result << std::endl;
  // std::cout << "f_vec_out = " << f_vec_out.transpose() << std::endl;

  #ifdef MPC_TIME_ALL
    std::cout << "QP took " << dur_qp_solve << " seconds." << std::endl; 
    std::cout << "QP rate " << (1.0/dur_qp_solve) << " Hz" << std::endl; 
  #endif
}

void CMPC::print_f_vec(int & n_Fr, const Eigen::VectorXd & f_vec_out){
  for(int i = 0; i < horizon; i++){
    std::cout << "horizon:" << i+1 << std::endl;

    for (int j = 0; j < n_Fr; j++){
      std::cout << "  f" << j << ":" << f_vec_out.segment(3*i*n_Fr + 3*j, 3).transpose() << std::endl;
    }
  }


}

void CMPC::solve_mpc(const Eigen::VectorXd & x0, const Eigen::VectorXd & X_des, const Eigen::MatrixXd & r_feet,
                     Eigen::VectorXd & x_pred, Eigen::VectorXd & f_vec_out){
  #ifdef MPC_TIME_ALL
    // Time the QP solve
    std::chrono::high_resolution_clock::time_point t_qp_all_start;
    std::chrono::high_resolution_clock::time_point t_qp_all_end;
    double dur_qp_all = 0.0;

    std::chrono::high_resolution_clock::time_point t_mat_setup_start;
    std::chrono::high_resolution_clock::time_point t_mat_setup_end;
    double dur_mat_setup = 0.0;
    // Start
    t_qp_all_start = std::chrono::high_resolution_clock::now();
    t_mat_setup_start = std::chrono::high_resolution_clock::now();
  #endif

  int n_Fr = r_feet.cols();

  // create continuous time state space matrices
  double psi_yaw = x0[2];
  Eigen::MatrixXd A(13,13); A.setZero();
  Eigen::MatrixXd B(13, 3*n_Fr); B.setZero();
  int n = A.cols();
  int m = B.cols(); 
  cont_time_state_space(x0, r_feet, A, B);

  // create discrete time state space matrices
  Eigen::MatrixXd Adt(n,n); Adt.setZero();
  Eigen::MatrixXd Bdt(n, m); Bdt.setZero();
  discrete_time_state_space(A, B, Adt, Bdt);

  // create A and B QP matrices
  Eigen::MatrixXd Aqp(n*horizon, n); Aqp.setZero();
  Eigen::MatrixXd Bqp(n*horizon, m*horizon); Bqp.setZero();
  qp_matrices(Adt, Bdt, Aqp, Bqp);

  // create force constraint matrices and vectors
  Eigen::MatrixXd CMat(6*n_Fr,3*n_Fr); // the QP constraint matrix for n_Fr reaction forces 
  Eigen::VectorXd cvec(6*n_Fr); // the constraint matrix for n_Fr reaction forces
  get_force_constraints(n_Fr, CMat, cvec);

  Eigen::MatrixXd Cqp(horizon*6*n_Fr, horizon*3*n_Fr);
  Eigen::VectorXd cvec_qp(horizon*6*n_Fr);
  get_qp_constraints(CMat, cvec, Cqp, cvec_qp);

  // Cost matrices (Values similar to cheetah 3)
  Eigen::MatrixXd Sqp(n*horizon, n*horizon);
  Eigen::MatrixXd Kqp(m*horizon, m*horizon);

  Eigen::VectorXd vecS_cost(n);
  //        <<  th1,  th2,  th3,  px,  py,  pz,   w1,  w2,   w3,   dpx,  dpy,  dpz,  g
  // vecS_cost << 0.25, 0.25, 10.0, 2.0, 2.0, 50.0, 0.0, 0.0, 0.30, 0.20, 0.2, 0.10, 0.0;
  vecS_cost << 10.0, 10.0, 100.0, 20.0, 20.0, 500.0, 0.5, 0.5, 3.0, 2.0, 2, 1.0, 0.0;
  Eigen::MatrixXd S_cost = vecS_cost.asDiagonal();
  double control_alpha = 1e-5; //4e-5
  get_qp_costs(n, m, vecS_cost, control_alpha, Sqp, Kqp);

  #ifdef MPC_TIME_ALL
    // End
    t_mat_setup_end = std::chrono::high_resolution_clock::now();
    dur_mat_setup = std::chrono::duration_cast< std::chrono::duration<double> >(t_mat_setup_end - t_mat_setup_start).count();

    std::cout << "QP matrices setup took " <<  dur_mat_setup << " seconds." << std::endl; 
    std::cout << "QP matrices setup rate " << (1.0/ dur_mat_setup) << " Hz" << std::endl; 

    std::cout << "robot state x_cur:" << x0.transpose() << std::endl;
    std::cout << "            x_des:" << X_des.head(n).transpose() << std::endl;
    std::cout << "horizon steps = " << horizon << ", horizon_time = " << horizon*mpc_dt << std::endl; 
  #endif

  // Solve MPC
  solve_mpc_qp(Aqp, Bqp, X_des, x0, Sqp, Kqp, Cqp, cvec_qp, f_vec_out);
  // Locally store the output force
  latest_f_vec_out = f_vec_out;

  // Compute prediction of state evolution after an interval of mpc_dt
  x_pred = (Aqp*x0 + Bqp*f_vec_out).head(n);

  #ifdef MPC_TIME_ALL
    // End
    t_qp_all_end = std::chrono::high_resolution_clock::now();
    dur_qp_all = std::chrono::duration_cast< std::chrono::duration<double> >(t_qp_all_end - t_qp_all_start).count();
    std::cout << "QP overall time took " <<  dur_qp_all << " seconds." << std::endl; 
    std::cout << "QP overall rate " << (1.0/ dur_qp_all) << " Hz" << std::endl; 
  #endif

  #ifdef MPC_TIME_ALL
    std::cout << "Predicted next state after dt = " << mpc_dt << " : "  << x_pred.transpose() << std::endl;
  #endif
}


void CMPC::get_constant_desired_x(const Eigen::VectorXd & x_des, Eigen::VectorXd & X_ref){
	X_ref = Eigen::VectorXd(horizon*13);
	X_ref.setZero();
	for(int i = 0; i < horizon; i++){
    X_ref.segment(13*i, 13) = x_des;
		// X_ref[13*i + 5] = x_des[5]; // Set constant desired z-height
	}

  #ifdef MPC_PRINT_ALL
  	std::cout << "X_ref " << std::endl;
  	std::cout << X_ref.transpose() << std::endl;
  #endif 
}




void CMPC::simulate_toy_mpc(){
  // System Params
  setRobotMass(50); // (kilograms) 
  Eigen::MatrixXd I_robot_body = 10.0*Eigen::MatrixXd::Identity(3,3); // Body Inertia matrix 
  // I_robot_body(0,0) = 07;
  // I_robot_body(1,1) = 0.26;
  // I_robot_body(2,2) = 0.242;
  setRobotInertia(I_robot_body);


  // MPC Params
  setHorizon(20); // horizon timesteps 
  setDt(0.025); // (seconds) per horizon
  setMu(0.9); //  friction coefficient
  setMaxFz(500); // (Newtons) maximum vertical reaction force per foot.


  // Starting robot state
  // Current reduced state of the robot
  // x = [Theta, p, omega, pdot, g] \in \mathbf{R}^13
  Eigen::VectorXd x0(13); 

  double init_roll(0), init_pitch(0), init_yaw(0), 
         init_com_x(0), init_com_y(0), init_com_z(0.25), 
         init_roll_rate(0), init_pitch_rate(0), init_yaw_rate(0),
         init_com_x_rate(0), init_com_y_rate(0), init_com_z_rate(0);

  x0 = getx0(init_roll, init_pitch, init_yaw,
            init_com_x, init_com_y, init_com_z,
            init_roll_rate, init_pitch_rate, init_yaw_rate,
            init_com_x_rate, init_com_y_rate, init_com_z_rate);

  // Feet Configuration
  // Set Foot contact locations w.r.t world
  Eigen::MatrixXd r_feet(3, 4); // Each column is a reaction force in x,y,z 
  r_feet.setZero();
  double foot_length = 0.05; // 5cm distance between toe and heel
  double nominal_width = 0.1; // 10cm distance between left and right feet

  // Flat ground contact, z = 0.0
  // 2 Contact Configuration
  // // Right Foot 
  // r_feet(0, 0) = 0.0; // x
  // r_feet(1, 0) = -nominal_width/2.0; //y
  // // Left Foot 
  // r_feet(0, 1) = 0.0;  // x
  // r_feet(1, 1) = nominal_width/2.0; //y

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
  get_constant_desired_x(x_des, X_des);

  Eigen::VectorXd x_pred(n); // Container to hold the predicted state after 1 horizon timestep 

  // Solve the MPC
  solve_mpc(x0, X_des, r_feet, x_pred, f_vec_out);
  print_f_vec(n_Fr, f_vec_out);
  // Populate force output from 1 horizon.
  assemble_vec_to_matrix(3, n_Fr, f_vec_out.head(3*n_Fr), f_Mat);

  // Simulate MPC for one time step
  double sim_dt = 1e-3;
  Eigen::VectorXd x_prev(n); x_prev = x0;
  Eigen::VectorXd x_next(n); x_next.setZero();
  integrate_robot_dynamics(sim_dt, x_prev, f_Mat, r_feet, x_next);

  // Simulate MPC for x seconds
  double total_sim_time = 7.0;
  int sim_steps = static_cast<int>(total_sim_time/sim_dt);
  std::cout << "sim_steps:" << sim_steps << std::endl;

  double last_control_time = 0.0;
  double cur_time = 0.0;

  std::cout << "x_start:" << x0.transpose() << std::endl;
  Eigen::VectorXd f_cmd(12); f_cmd.setZero();
  
  bool print_for_plots = false;

  if (print_for_plots){
    printf("t, r, p, y, x, y, z, wx, wy, wz, dx, dy, dz, f0x, f0y, f0z, f1x, f1y, f1z, f2x, f2y, f2z, f3x, f3y, f3z\n");
  }
  for(int i = 0; i < sim_steps; i++){
    // Solve new MPC every mpc control tick
    if ((cur_time - last_control_time) > mpc_dt){

      solve_mpc(x_prev, X_des, r_feet, x_pred, f_vec_out);      
      assemble_vec_to_matrix(3, n_Fr, f_vec_out.head(3*n_Fr), f_Mat);
      last_control_time = cur_time;      
      f_cmd.head(m) = f_vec_out.head(m);

      if (!print_for_plots){      
        std::cout << "  Computed MPC Forces" << std::endl;
        printf("  f0:(%0.3f,%0.3f,%0.3f)\n", f_cmd[0], f_cmd[1], f_cmd[2]);
        printf("  f1:(%0.3f,%0.3f,%0.3f)\n", f_cmd[3], f_cmd[4], f_cmd[5]);
        if (n_Fr > 2){
          printf("  f2:(%0.3f,%0.3f,%0.3f)\n", f_cmd[6], f_cmd[7], f_cmd[8]);
          printf("  f3:(%0.3f,%0.3f,%0.3f)\n", f_cmd[9], f_cmd[10], f_cmd[11]);
        }
      }

    }

    // Integrate the robot dynamics
    integrate_robot_dynamics(sim_dt, x_prev, f_Mat, r_feet, x_next);
    // Update x
    x_prev = x_next;
    // Increment time
    cur_time += sim_dt;

    if (print_for_plots){
      printf("%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f,%0.3f,%0.3f, %0.3f,%0.3f,%0.3f, %0.3f,%0.3f,%0.3f, %0.3f,%0.3f,%0.3f \n", 
                                                                                                                cur_time, 
                                                                                                                x_next[0], x_next[1], x_next[2], 
                                                                                                                x_next[3], x_next[4], x_next[5],
                                                                                                                x_next[6], x_next[7], x_next[8],
                                                                                                                x_next[9], x_next[10], x_next[11],
                                                                                                                f_cmd[0], f_cmd[1], f_cmd[2],
                                                                                                                f_cmd[3], f_cmd[4], f_cmd[5],
                                                                                                                f_cmd[6], f_cmd[7], f_cmd[8],
                                                                                                                f_cmd[9], f_cmd[10], f_cmd[11]);
    }else{
      printf("t:%0.3f, (R,P,Y):(%0.3f, %0.3f, %0.3f), (x,y,z):(%0.3f, %0.3f, %0.3f), (wx, wy, wz):(%0.3f, %0.3f, %0.3f), (dx, dy, dz):(%0.3f, %0.3f, %0.3f) \n", 
                                                                                                                cur_time, 
                                                                                                                x_next[0], x_next[1], x_next[2], 
                                                                                                                x_next[3], x_next[4], x_next[5],
                                                                                                                x_next[6], x_next[7], x_next[8],
                                                                                                                x_next[9], x_next[10], x_next[11]);
    }

    // std::cout << "    f:" << f_vec_out.head(m).transpose() << std::endl;      
    // for(int j = 0; j < n_Fr; j++){
    //   std::cout << "    f" << j << ":" << f_vec_out.segment(3*j, 3).transpose() << std::endl;      
    // }
    // std::cout << "cur_time:" << cur_time << "  last_control_time:" << last_control_time << std::endl;
  }
  std::cout << "x_des" << X_des.tail(n).transpose() << std::endl; 


}