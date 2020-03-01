#include <PnC/DracoPnC/PredictionModule/DCMWalkingReferenceTrajectoryModule.hpp>

// Initialize by assigning the contact indices to a robot side.
DCMWalkingReferenceTrajectoryModule::DCMWalkingReferenceTrajectoryModule
	(const std::vector<int> & index_to_side_in): WalkingReferenceTrajectoryModule(index_to_side_in){

}

DCMWalkingReferenceTrajectoryModule::~DCMWalkingReferenceTrajectoryModule(){

}

void DCMWalkingReferenceTrajectoryModule::setFootsteps(double t_walk_start_in, const std::vector<DracoFootstep> & footstep_list_in){
	dcm_reference.initialize_footsteps_rvrp(footstep_list_, left_foot_start_, right_foot_start_, x_com_start_);
}

// gets the references 
int DCMWalkingReferenceTrajectoryModule::getState(const double time){

}
void DCMWalkingReferenceTrajectoryModule::getMPCRefComAndOri(const double time, Eigen::Vector3d & x_com_out, Eigen::Quaterniond & x_ori_out){

}

double DCMWalkingReferenceTrajectoryModule::getMaxNormalForce(int index, double time){

}

// If true, populates the new footstep landing location
// If false, the MPC should use the current location of the foot
bool DCMWalkingReferenceTrajectoryModule::getFutureMPCFootstep(double time, DracoFootstep & footstep_landing_location){

}

// set that a particular contact was hit early
// index: DRACO_LEFT_FOOTSTEP or DRACO_RIGHT_FOOTSTEP
// time: time of early contact
void DCMWalkingReferenceTrajectoryModule::setEarlyFootContact(const int index, const double time){

}

// set that a particular foot was hit early. automatically handles the contact updates
// robot_side DRACO_LEFT_FOOTSTEP or DRACO_RIGHT_FOOTSTEP
// time: time of early contact
void DCMWalkingReferenceTrajectoryModule::setEarlyFootSideContact(const int robot_side, const double time){

}

// helper function to identify which footstep is in swing
// if false. the foot is in not in swing for the time queried
bool DCMWalkingReferenceTrajectoryModule::whichFootstepIndexInSwing(const double time, int & footstep_index){

}