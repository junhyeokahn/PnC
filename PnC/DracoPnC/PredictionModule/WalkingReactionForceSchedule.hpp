#include <PnC/DracoPnC/PredictionModule/DracoFootstep.hpp>
#include <PnC/MPC/ReactionForceSchedule.hpp>

#include <vector>
#include <map>


class WalkingReactionForceSchedule : public ReactionForceSchedule {
public:
	WalkingReactionForceSchedule();
	virtual ~WalkingReactionForceSchedule();

	// Set contact indices to a robot side
	void setContactIndexToSide(const std::vector<int> & index_to_side_in);
	// Set the footsteps for the walking reaction force schedule
	void setFootsteps(const double t_walk_start_in, const std::vector<DracoFootstep> & footstep_list_in);

	// default is to return the max z force
	virtual double getMaxNormalForce(const int index, const double time);

private:
	double t_walk_start_ = 0.0;

	// keep track of indices to robot side
	std::vector<int> index_to_side_;

	// List of footsteps
	std::vector<DracoFootstep> footstep_list_; // list of footsteps
	std::map<int, double> early_contact_times_;
};
