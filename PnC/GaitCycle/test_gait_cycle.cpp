#include <PnC/GaitCycle/GaitCycle.hpp>

int main(int argc, char** argv) {
	double swing_time = 0.25;
	double transition_time = 0.125;
	double biped_walking_offset = swing_time + transition_time;
	double total_gait_duration = 2.0*swing_time + 2.0*transition_time;

	GaitCycle gait_cycle(swing_time, total_gait_duration, {0.0, biped_walking_offset});

	// Test member functions
	double start_time = 0.0;
	double time = 0.0;
	double offset = 0.0;
	double bounded_phase_value = gait_cycle.getGaitPhaseValue(start_time, time, offset);

	std::cout << "  start_time: " << start_time << std::endl;
	std::cout << "  time: " << time << std::endl;
	std::cout << "  offset: " << offset << std::endl;
	std::cout << "  bounded_phase_value: " << bounded_phase_value << std::endl;

	int contact_state = gait_cycle.getContactStateGivenPhaseValue(bounded_phase_value);
	std::cout << "  contact state: " << contact_state << std::endl;	
	gait_cycle.printCurrentGaitInfo();

	// Test Gait Cycle internal state with two contacts
	start_time = 0.0;
	time = transition_time + biped_walking_offset;
	std::cout << "Update to (t0, t) = (" << start_time << ", " << time << ")" << std::endl;
	gait_cycle.updateContactStates(start_time, time);
	gait_cycle.printCurrentGaitInfo();

	// Test Gait Cycle internal state with four contacts
	std::cout << "\n Set new gait with 4 contact points" << std::endl;
	gait_cycle.setGaitOffsets({0.0, 0.0, biped_walking_offset, biped_walking_offset});
	start_time = 0.0;
	time = transition_time + swing_time*0.5;
	std::cout << "Update to (t0, t) = (" << start_time << ", " << time << ")" << std::endl;
	gait_cycle.updateContactStates(start_time, time);
	gait_cycle.printCurrentGaitInfo();


	return 0;
}
