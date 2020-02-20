#include <PnC/DracoPnC/PredictionModule/WalkingReactionForceSchedule.hpp>

WalkingReactionForceSchedule::WalkingReactionForceSchedule(): 
	ReactionForceSchedule(){
	t_walk_start = 0.0;
}

WalkingReactionForceSchedule::~WalkingReactionForceSchedule(){

}

// default is to return the max z force
double WalkingReactionForceSchedule::getMaxNormalForce(int index, double time){
	// if index is out of bounds or time requested is earlier than 
	if 	( ((index < 0) || (index >= footstep_list.size())) ||
		  (time <= t_walk_start) ){
		return default_max_z_force;
	}
}

// Set the footsteps for the walking reaction force schedule
void WalkingReactionForceSchedule::setFootsteps(double t_walk_start_in, std::vector<DracoFootstep> & footstep_list_in){
	t_walk_start = t_walk_start_in;
	// Clear internal data then copy footsteps
	footstep_list.clear();
	early_contact_times.clear();
	for(int i = 0; i < footstep_list_in.size(); i++){
		footstep_list.push_back(footstep_list_in[i]);
	}
}