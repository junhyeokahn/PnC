#include <PnC/GaitCycle/GaitCycle.hpp>

int main(int argc, char** argv) {
	double swing_time = 0.25;
	double transition_time = 0.125;
	double total_gait_duration = 2.0*swing_time + 2.0*transition_time;
	GaitCycle gait_cycle(swing_time, total_gait_duration, {0.0, transition_time});


	double start_time = 0.25;
	double time = 0.325;
	double offset = 0.0;
	gait_cycle.getGaitPhaseValue(start_time, time, offset);

	return 0;
}
