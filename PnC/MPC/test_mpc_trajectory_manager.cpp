#include <PnC/MPC/MPCDesiredTrajectoryManager.hpp>
#include <Eigen/Dense>

double cube_func(double a, double t){
	return a*std::pow(t,3);
}

double cube_func_deriv(double a, double t){
	return 3*a*std::pow(t,2);
}

int main(int argc, char const ** argv){
	int state_size = 13;
	int horizon = 40;
	double mpc_dt = 0.025;
	MPCDesiredTrajectoryManager trajectory_manager(state_size, horizon, mpc_dt);

	double t = 0.0;
	double t_total = 45*mpc_dt;
	double dt = 0.01;
	int n = static_cast<int>(t_total/dt);


	// // Set the starting state of the system
	Eigen::VectorXd X_start(state_size); X_start.setZero();
	X_start[12] = -9.81;
	// // Set prediction 
	Eigen::VectorXd X_pred(state_size*horizon); X_pred.setZero();
	for(int i = 0; i < horizon; i++){
		// RPY
		X_pred[i*state_size + 0] = cube_func(1,(i+1)*mpc_dt);
		X_pred[i*state_size + 1] = 2.0*cube_func(1,(i+1)*mpc_dt); 
		X_pred[i*state_size + 2] = 3.0*cube_func(1,(i+1)*mpc_dt); 

		// XYZ
		X_pred[i*state_size + 3] = 4.0*cube_func(1,(i+1)*mpc_dt); 
		X_pred[i*state_size + 4] = 5.0*cube_func(1,(i+1)*mpc_dt); 
		X_pred[i*state_size + 5] = 6.0*cube_func(1,(i+1)*mpc_dt);

		// RPY rates
		X_pred[i*state_size + 6] = 1.0*cube_func_deriv(1,(i+1)*mpc_dt); 
		X_pred[i*state_size + 7] = 2.0*cube_func_deriv(1,(i+1)*mpc_dt); 
		X_pred[i*state_size + 8] = 3.0*cube_func_deriv(1,(i+1)*mpc_dt);

		// xyz velocities
		X_pred[i*state_size + 9] = 4.0*cube_func_deriv(1,(i+1)*mpc_dt); 
		X_pred[i*state_size + 10] = 5.0*cube_func_deriv(1,(i+1)*mpc_dt); 
		X_pred[i*state_size + 11] = 6.0*cube_func_deriv(1,(i+1)*mpc_dt);
	}

	for(int i = 0; i < horizon; i++){
	std::cout << i << ": X_pred = " << X_pred.segment(i*state_size, state_size).transpose() << std::endl;		
	}


	// Perform fit
	double time_start = 0.0;
	std::cout << "Pre fit" << std::endl;
    trajectory_manager.setStateKnotPoints(time_start, X_start, X_pred); 
	std::cout << "Post fit" << std::endl;

	Eigen::VectorXd x_traj; 
	for(int i = 0; i < n+1; i++){
		t = i*dt;
		trajectory_manager.getState(t, x_traj);
		std::cout << "t:" << t << " : " << x_traj.transpose() << std::endl;
	}




	return 0;
}
