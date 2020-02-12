#ifndef H_GAIT_CYCLE
#define H_GAIT_CYCLE

// Standard
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <cmath>

// Header
class GaitCycle{
public: 
	// swing_time_in: length of flight time of a particular contact point in seconds
	// total_gait_duration_in: total time of the the gait in seconds 
	// gait_offsets_in: time offsets in seconds to be used for the phase of a contact point. 
	GaitCycle(const double swing_time_in, const double total_gait_duration_in, const std::vector<double> gait_offsets_in);
	~GaitCycle();

	double getGaitPhaseValue(double start_time, double time, double offset);

	void setSwingTime(const double swing_time_in);
	void setTotalGaitDuration(const double total_gait_duration_in);

private:
	std::vector<double> m_gait_offsets;
	double m_total_gait_duration;
	double m_swing_time;

};

#endif