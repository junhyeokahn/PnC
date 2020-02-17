#include <PnC/MPC/MPCDesiredTrajectoryManager.hpp>

int main(int argc, char const ** argv){
	int state_size = 13;
	int horizon = 15;
	MPCDesiredTrajectoryManager trajectory_manager(state_size, horizon);
	return 0;
}
