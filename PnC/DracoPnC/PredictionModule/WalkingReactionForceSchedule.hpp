#ifndef WALKING_REACTION_FORCE_SCHEDULE_H
#define WALKING_REACTION_FORCE_SCHEDULE_H

#include <PnC/DracoPnC/PredictionModule/WalkingReferenceTrajectoryModule.hpp>
#include <PnC/DracoPnC/PredictionModule/DracoFootstep.hpp>
#include <PnC/MPC/ReactionForceSchedule.hpp>

#include <vector>
#include <map>


class WalkingReferenceTrajectoryModule;

class WalkingReactionForceSchedule : public ReactionForceSchedule {
public:
	WalkingReactionForceSchedule(WalkingReferenceTrajectoryModule* reference_traj_module_in);
	virtual ~WalkingReactionForceSchedule();

	WalkingReferenceTrajectoryModule* reference_traj_module;

	// default is to return the max z force
	virtual double getMaxNormalForce(const int index, const double time);

	void testFunction();

};

#endif