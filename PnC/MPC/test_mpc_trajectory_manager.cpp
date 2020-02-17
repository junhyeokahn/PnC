#include <PnC/MPC/MPCDesiredTrajectoryManager.hpp>

int main(int argc, char const ** argv){
	int state_size = 13;
	int horizon = 15;
	double mpc_dt = 0.025;
	MPCDesiredTrajectoryManager trajectory_manager(state_size, horizon, mpc_dt);

	double t = 0.0;
	double t_total = 0.15;
	double dt = 0.003;
	int n = static_cast<int>(t_total/dt);

	for(int i = 0; i < n+1; i++){
		t = i*dt;
		// std::cout << "t:" << t << ", ";
		trajectory_manager.getPos(t);
	}



	return 0;
}
