#include <PnC/DracoPnC/PredictionModule/WalkingReactionForceSchedule.hpp>

WalkingReactionForceSchedule::WalkingReactionForceSchedule(WalkingReferenceTrajectoryModule* reference_traj_module_in): 
	ReactionForceSchedule(){
	reference_traj_module = reference_traj_module_in;
}

WalkingReactionForceSchedule::~WalkingReactionForceSchedule(){
}

void WalkingReactionForceSchedule::testFunction(){
	std::cout << "hello size of footstep list is " << reference_traj_module->footstep_list_.size() << std::endl;

}

// default is to return the max z force
double WalkingReactionForceSchedule::getMaxNormalForce(const int index, const double time){
	// if index is out of bounds or time requested is earlier than 
	if 	( ((index < 0) || (index >= reference_traj_module->footstep_list_.size())) ||
		  (time <= reference_traj_module->t_walk_start_) ){
		return default_max_z_force_;
	}
}




