#include <PnC/DracoPnC/PredictionModule/WalkingReferenceTrajectoryModule.hpp>

WalkingReferenceTrajectoryModule::WalkingReferenceTrajectoryModule(){
	t_walk_start_ = 0.0;
	reaction_force_schedule_ptr.reset(new WalkingReactionForceSchedule());
	walking_rfs_ptr = std::static_pointer_cast<WalkingReactionForceSchedule>(reaction_force_schedule_ptr);

}

WalkingReferenceTrajectoryModule::~WalkingReferenceTrajectoryModule(){}


// set the starting configuration
void WalkingReferenceTrajectoryModule::setStartingConfiguration(const Eigen::Vector3d x_com_start_in,
							  const Eigen::Quaterniond x_ori_start_in,
							  const DracoFootstep & left_foot_start_in, 
							  const DracoFootstep & right_foot_start_in){
	x_com_start_ = x_com_start_in;
	x_ori_start_ = x_ori_start_in;
	left_foot_start_ = left_foot_start_in;
	right_foot_start_ = right_foot_start_in;
}

//set the footsteps and starting time to walk
void WalkingReferenceTrajectoryModule::setFootsteps(double t_walk_start_in, const std::vector<DracoFootstep> & footstep_list_in){
	t_walk_start_ = t_walk_start_in;

	// Clear internal data then copy footsteps
	footstep_list_.clear();
	early_contact_times_.clear();
	for(int i = 0; i < footstep_list_in.size(); i++){
		footstep_list_.push_back(footstep_list_in[i]);
	}

	// set footsteps on the reaction force schedule
	walking_rfs_ptr->setFootsteps(t_walk_start_in, footstep_list_in);

}

// gets the references 
int WalkingReferenceTrajectoryModule::getState(const double time){

}
void WalkingReferenceTrajectoryModule::getMPCRefCom(const double time, Eigen::Vector3d x_com){

}
void WalkingReferenceTrajectoryModule::getMPCRefOri(const double time, Eigen::Quaterniond x_ori){

}
double WalkingReferenceTrajectoryModule::getMaxNormalForce(int index, double time){

}

// set that a particular contact was hit early
// index: DRACO_LEFT_FOOTSTEP or DRACO_RIGHT_FOOTSTEP
// time: time of early contact
void WalkingReferenceTrajectoryModule::setEarlyFootContact(const int index, const double time){
}