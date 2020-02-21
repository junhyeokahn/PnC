#include <PnC/DracoPnC/PredictionModule/WalkingReactionForceSchedule.hpp>

WalkingReactionForceSchedule::WalkingReactionForceSchedule(): 
	ReactionForceSchedule(){
	t_walk_start_ = 0.0;
}

WalkingReactionForceSchedule::~WalkingReactionForceSchedule(){

}

void WalkingReactionForceSchedule::setContactIndexToSide(const std::vector<int> & index_to_side_in){
    index_to_side_ = index_to_side_in;
}

// default is to return the max z force
double WalkingReactionForceSchedule::getMaxNormalForce(const int index, const double time){
	// if index is out of bounds or time requested is earlier than 
	if 	( ((index < 0) || (index >= footstep_list_.size())) ||
		  (time <= t_walk_start_) ){
		return default_max_z_force_;
	}
}

// Set the footsteps for the walking reaction force schedule
void WalkingReactionForceSchedule::setFootsteps(const double t_walk_start_in, const std::vector<DracoFootstep> & footstep_list_in){
	t_walk_start_ = t_walk_start_in;
	// Clear internal data then copy footsteps
	footstep_list_.clear();
	early_contact_times_.clear();
	for(int i = 0; i < footstep_list_in.size(); i++){
		footstep_list_.push_back(footstep_list_in[i]);
	}
}


