#ifndef REACTION_FORCE_SCHEDULE_H
#define REACTION_FORCE_SCHEDULE_H

#include <stdio.h>

class ReactionForceSchedule{
public:
	ReactionForceSchedule();
	virtual ~ReactionForceSchedule();

	// Sets the default maximum reaction force
	void setDefaultMaxNormalForce(double default_max_z_force_in);

	// default is to return the max z force
	virtual double getMaxNormalForce(int index, double time);

	double getDefaultMaxNormalForce();

	// returns a value between 0.0 and 1.0
	// computes getMaxNormalForce(index, time)/default_max_z_force
	double getTransitionVariable(int index, double time);

	// Default max z force
	double default_max_z_force_ = 500;
};


#endif