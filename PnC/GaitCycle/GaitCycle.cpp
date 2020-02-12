#include <PnC/GaitCycle/GaitCycle.hpp>

// 
GaitCycle::GaitCycle(const double swing_time_in, const double total_gait_duration_in, const std::vector<double> gait_offsets_in){
	// Set Gait Offsets
	if (gait_offsets_in.size() == 0){
		std::cerr << "[GaitCycle] Warning. Size of gait offsets is 0." << std::endl;
	}
	m_gait_offsets.clear();
	for(int i = 0; i < gait_offsets_in.size(); i++){
		m_gait_offsets.push_back(gait_offsets_in[i]);
	}

	// Set Swing Time
	setSwingTime(swing_time_in);
	setTotalGaitDuration(total_gait_duration_in);
	if (m_swing_time > m_total_gait_duration){
		std::cerr << "[GaitCycle] Warning. Swing time is larger than the total gait duration." << std::endl;
	}
	std::cout << "[GaitCycle] Object Constructed." << std::endl;	
}

GaitCycle::~GaitCycle(){
	std::cout << "[GaitCycle] Destroyed" << std::endl;	
}

double GaitCycle::getGaitPhaseValue(double start_time, double time, double offset){
	double time_local = (time - start_time) + offset;
	double phase_value = time_local / m_total_gait_duration;

	// Wrap Phase Value
	double bounded_phase_value = std::fmod(phase_value, 1.0);

	std::cout << "  start_time: " << start_time << std::endl;
	std::cout << "  time: " << time << std::endl;
	std::cout << "  offset: " << offset << std::endl;
	std::cout << "  time_local: " << time_local << std::endl;

	std::cout << "  phase_value: " << phase_value << std::endl;

	if (bounded_phase_value > 0.0){
		std::cout << "  bounded_phase_value: " << bounded_phase_value << std::endl;
		return bounded_phase_value;
	}else{
		// time_local is negative (looking back into the past return 1.0 - bounded_phase_value) 
		std::cout << "  bounded_phase_value: " << (1.0 + bounded_phase_value) << std::endl;
		return (1.0 + bounded_phase_value);
	}


}


void GaitCycle::setSwingTime(const double swing_time_in){
	m_swing_time = swing_time_in;
	std::cout << "[GaitCycle] Swing time is set to: " << m_swing_time << std::endl;
}
void GaitCycle::setTotalGaitDuration(const double total_gait_duration_in){
	m_total_gait_duration = total_gait_duration_in;
	std::cout << "[GaitCycle] Total gait duration is set to: " << m_total_gait_duration << std::endl;
}
