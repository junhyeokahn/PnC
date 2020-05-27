#include <PnC/ValkyriePnC/TrajectoryManagers/DCMPlannerTrajectoryManager.hpp>

DCMPlannerTrajectoryManager::DCMPlannerTrajectoryManager(DCMPlanner* _dcm_planner, RobotSystem* _robot) : TrajectoryManagerBase(_robot){
	dcm_planner_ = _dcm_planner;
}

DCMPlannerTrajectoryManager::~DCMPlannerTrajectoryManager(){
}

void DCMPlannerTrajectoryManager::paramInitialization(const YAML::Node& node){

}

void DCMPlannerTrajectoryManager::incrementStepIndex(){
	current_footstep_index_++;
}
void DCMPlannerTrajectoryManager::resetStepIndex(){
   current_footstep_index_ = 0;
}


// Updates the feet pose of the starting stance 
void DCMPlannerTrajectoryManager::updateStartingStance(){
    Eigen::Vector3d lfoot_pos = robot_->getBodyNodeCoMIsometry(ValkyrieBodyNode::leftCOP_Frame).translation();
    Eigen::Quaterniond lfoot_ori(robot_->getBodyNodeCoMIsometry(ValkyrieBodyNode::leftCOP_Frame).linear());
    left_foot_start_.setPosOriSide(lfoot_pos, lfoot_ori, LEFT_ROBOT_SIDE);

    Eigen::Vector3d rfoot_pos = robot_->getBodyNodeCoMIsometry(ValkyrieBodyNode::rightCOP_Frame).translation();
    Eigen::Quaterniond rfoot_ori(robot_->getBodyNodeCoMIsometry(ValkyrieBodyNode::rightCOP_Frame).linear());
    right_foot_start_.setPosOriSide(rfoot_pos, rfoot_ori, RIGHT_ROBOT_SIDE);
}

// Updates the local footstep list (ie: footstep preview) for trajectory generation:
void DCMPlannerTrajectoryManager::updatePreview(const int max_footsteps_to_preview, std::vector<Footstep> & footstep_list){
	footstep_preview_list_.clear();
	for(int i = 0; i < max_footsteps_to_preview; i++){
		if ((i + current_footstep_index_) < footstep_list.size()){
			footstep_preview_list_.push_back(footstep_list[i + current_footstep_index_]);			
		}else{
			break;
		}
	}

	// Identify stance leg from footstep.
	// int stance_side = LEFT_ROBOT_SIDE;
	// int stance_body_id = ValkyrieBodyNode::leftCOP_Frame;
	// if (footstep_list[0].robot_side == LEFT_ROBOT_SIDE){
	// 	stance_side = RIGHT_ROBOT_SIDE;
	// 	stance_body_id = ValkyrieBodyNode::rightCOP_Frame;
	// }
}

// Footstep sequence primitives -----------------------------------------------------------
// Creates footstep in place
void DCMPlannerTrajectoryManager::populateStepInPlace(const int num_steps, const int robot_side_first, std::vector<Footstep> & footstep_list){
	updateStartingStance(); // Update the starting foot locations of the robot

	int robot_side = robot_side_first;
	for(int i = 0; i < num_steps; i++){
		// Add in place step and switch sides
		if (robot_side == LEFT_ROBOT_SIDE){
			footstep_list.push_back(left_foot_start_);
			robot_side = RIGHT_ROBOT_SIDE;
		}else{
			footstep_list.push_back(right_foot_start_);
			robot_side = LEFT_ROBOT_SIDE;			
		}
	}
}

// Populates the input footstep list with a predefined walking forward behavior
void DCMPlannerTrajectoryManager::populateWalkForward(const int num_steps,
							 						  const double nominal_step_forward_distance,
							 						  const double nominal_step_width_distance,
													  const double midstep_distance_multiplier,
													  std::vector<Footstep> & footstep_list){

}

// Populates the input footstep list with a predefined footstep list to turn left
void DCMPlannerTrajectoryManager::populateTurnLeft(const double turn_angle,
							  					   const double nominal_step_width_distance,
												   std::vector<Footstep> & footstep_list){

}

// Populates the input footstep list with a predefined footstep list to turn right
void DCMPlannerTrajectoryManager::populateTurnRight(const double turn_angle,
								  				    const double nominal_step_width_distance,
													std::vector<Footstep> & footstep_list){	
}