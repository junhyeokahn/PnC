#ifndef H_GAIT_CYCLE
#define H_GAIT_CYCLE

// Standard
#include <math.h>
#include <stdio.h>
#include <string>
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

	// start_time in seconds. 
	// time in seconds. the current time.
	// offset in seconds
	// Returns the gait phase value \in [0, 1) given the start time, current time, and the time offset
	double getGaitPhaseValue(const double start_time, const double time, const double offset);
	
	// phase_value \in [0, 1)
	// Returns 1 if the contact point is active. 0 otherwise.
	//	   ignoring wrap arounds and swing time < gait duration time, the convention is that with no offsets, 
	//	   if (gait duration-swing time) <= t-t0 <= gait duration, then the contact point is NOT active
	//	   where t is the current time and t0 is the start time.
	//     considering wrap-arounds, equivalently, 
	//	   if the phase variable, phi satisfies: phi >= (gait duration - swing time)/gait duration, then the contact point is not active
	int getContactStateGivenPhaseValue(const double phase_value);

	// Updates an internal model of the contact states of the gait
	void updateContactStates(const double start_time, const double time);

	// Get the state of the contact based on the index.
	int getContactState(int index);

	// Set swing time for a contact point of the gait.
	void setSwingTime(const double swing_time_in);
	// set the total gait duration
	void setTotalGaitDuration(const double total_gait_duration_in);
	// sets a new gait offsets 
	void setGaitOffsets(const std::vector<double> gait_offsets_in); 

	// prints the offset and the current states of the gait
	void printCurrentGaitInfo();

private:
	int m_num_contact_points; // internal variable that keeps track of the number of contact points
	double m_time; // internal variable that keep track of the current time of the gait
	double m_start_time; // internal variable that keep track of the start time of the gait
	double m_flight_phase; // value of phase when flight occurs

	std::vector<int> m_internal_gait_contact_states; // a vector of the contact states
	std::vector<double> m_internal_gait_phase_states; // a vector of the phase states of the contacts 
	std::vector<double> m_gait_offsets; // a vector of the offset times for the contacts

	double m_total_gait_duration;
	double m_swing_time;

};

#endif