#include <PnC/DracoPnC/PredictionModule/DracoFootstep.hpp>
#include <PnC/MPC/ReactionForceSchedule.hpp>

#include <vector>
#include <map>


class WalkingReactionForceSchedule : public ReactionForceSchedule {
public:
	WalkingReactionForceSchedule();
	virtual ~WalkingReactionForceSchedule();

	// default is to return the max z force
	virtual double getMaxNormalForce(int index, double time);

	// Set the footsteps for the walking reaction force schedule
	void setFootsteps(double t_walk_start_in, std::vector<DracoFootstep> & footstep_list_in);

private:
	double t_walk_start = 0.0;

	// List of footsteps
	std::vector<DracoFootstep> footstep_list; // list of footsteps
	std::map<int, double> early_contact_times;
};
