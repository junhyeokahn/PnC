#include <PnC/ValkyriePnC/TrajectoryManagers/DCMPlannerTrajectoryManager.hpp>

DCMPlannerTrajectoryManager::DCMPlannerTrajectoryManager(DCMPlanner* _dcm_planner, RobotSystem* _robot) : TrajectoryManagerBase(_robot){
    myUtils::pretty_constructor(2, "TrajectoryManager: DCM Planner");
  dcm_planner_ = _dcm_planner;
}

DCMPlannerTrajectoryManager::~DCMPlannerTrajectoryManager(){
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
    left_foot_stance_.setPosOriSide(lfoot_pos, lfoot_ori, LEFT_ROBOT_SIDE);

    Eigen::Vector3d rfoot_pos = robot_->getBodyNodeCoMIsometry(ValkyrieBodyNode::rightCOP_Frame).translation();
    Eigen::Quaterniond rfoot_ori(robot_->getBodyNodeCoMIsometry(ValkyrieBodyNode::rightCOP_Frame).linear());
    right_foot_stance_.setPosOriSide(rfoot_pos, rfoot_ori, RIGHT_ROBOT_SIDE);
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
}

bool DCMPlannerTrajectoryManager::initialize(const double t_walk_start_in, const std::vector<Footstep> & footstep_list_in,
                                             const double t_transfer_in,
                                             const Eigen::Quaterniond & ori_start_in,
                                             const Eigen::Vector3d & dcm_pos_start_in, 
                                             const Eigen::Vector3d & dcm_vel_start_in){

  if (footstep_list_in.size() == 0){
    return false;
  }

  t_walk_start_ = t_walk_start_in;
  // Clear internal data then copy footsteps
  footstep_list_copy_.clear();
  for (int i = current_footstep_index_; i < footstep_list_in.size(); i++){
    footstep_list_copy_.push_back(footstep_list_in[i]);
  }

  // Reset index
  resetStepIndex();
  updateStartingStance();
  left_foot_start_ = left_foot_stance_;
  right_foot_start_ = right_foot_stance_;
  updatePreview(4, footstep_list_copy_);

    // Set DCM reference
  dcm_planner_->setInitialTime(t_walk_start_);
  dcm_planner_->setInitialOri(ori_start_in);
  dcm_planner_->initialize_footsteps_rvrp(footstep_preview_list_, left_foot_start_, right_foot_start_, 
                      dcm_pos_start_in, dcm_vel_start_in);
  // Set transfer time
  dcm_planner_->t_transfer = t_transfer_in;

  // Initialization successful
  return true;
}

// Footstep sequence primitives -----------------------------------------------------------
// Creates footstep in place
void DCMPlannerTrajectoryManager::populateStepInPlace(const int num_steps, const int robot_side_first, std::vector<Footstep> & footstep_list){
  updateStartingStance(); // Update the starting foot locations of the robot

  int robot_side = robot_side_first;
  for(int i = 0; i < num_steps; i++){
    // Add in place step and switch sides
    if (robot_side == LEFT_ROBOT_SIDE){
      footstep_list.push_back(left_foot_stance_);
      robot_side = RIGHT_ROBOT_SIDE;
    }else{
      footstep_list.push_back(right_foot_stance_);
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


void DCMPlannerTrajectoryManager::paramInitialization(const YAML::Node& node){
  // void setCoMHeight(double z_vrp_in); // Sets the desired CoM Height
}
