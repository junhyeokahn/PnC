#ifndef DCM_WALKING_REACTION_FORCE_SCHEDULE_H
#define DCM_WALKING_REACTION_FORCE_SCHEDULE_H

#include <PnC/DracoPnC/PredictionModule/DCMWalkingReferenceTrajectoryModule.hpp>
#include <PnC/DracoPnC/PredictionModule/DracoFootstep.hpp>
#include <PnC/MPC/ReactionForceSchedule.hpp>

#include <vector>
#include <map>


class DCMWalkingReferenceTrajectoryModule;

class DCMWalkingReactionForceSchedule : public ReactionForceSchedule {
public:
	DCMWalkingReactionForceSchedule(DCMWalkingReferenceTrajectoryModule* reference_traj_module_in);
	virtual ~DCMWalkingReactionForceSchedule();

	DCMWalkingReferenceTrajectoryModule* reference_traj_module;

	// default is to return the max z force
	virtual double getMaxNormalForce(const int index, const double time);

	void testFunction();

private:
	double clampMaxFz(double Fz_in);

};

#endif