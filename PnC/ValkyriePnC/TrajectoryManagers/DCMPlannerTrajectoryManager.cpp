#include <PnC/ValkyriePnC/TrajectoryManagers/DCMPlannerTrajectoryManager.hpp>

DCMPlannerTrajectoryManager::DCMPlannerTrajectoryManager(DCMPlanner* _dcm_planner, RobotSystem* _robot) : TrajectoryManagerBase(_robot){
    myUtils::pretty_constructor(2, "TrajectoryManager: DCM Planner");
  dcm_planner_ = _dcm_planner;

  // Initialize step index
  resetStepIndex();

  // Initialize default parameters
  t_additional_init_transfer_ = 0.0;        
  t_contact_transition_ = 0.45;  
  t_swing_ = 1.0;               

  // DCM walking parameters
  percentage_settle_ = 0.99; // percent to converge at the end of the trajectory
  alpha_ds_ = 0.5; // value between 0.0 and 1.0 for double support DCM interpolation
  nominal_com_height_ = 1.015; // vertical m from stance foot

  convertTemporalParamsToDCMParams();
}

DCMPlannerTrajectoryManager::~DCMPlannerTrajectoryManager(){
}

void DCMPlannerTrajectoryManager::convertTemporalParamsToDCMParams(){
  // Fixed transforms
  t_ds_ = t_contact_transition_; // double support polynomial transfer time
  t_ss_ = t_swing_; // single support exponential interpolation  time
  // polynomial interpolation time during contact transition: t_transfer + t_ds + (1-alpha*t_ds).
  t_transfer_init_ = t_additional_init_transfer_ ; // additional transfer time offset
  t_transfer_mid_ = (alpha_ds_-1.0)*t_ds_;  // transfer time offset for midstep transfers
}


double DCMPlannerTrajectoryManager::getInitialContactTransferTime(){
  double t_initial_transfer_time = t_additional_init_transfer_ + t_ds_ + (1-alpha_ds_)*t_ds_; // the total initial transfer time before the foot swinng
  return t_initial_transfer_time;
}

double DCMPlannerTrajectoryManager::getMidStepContactTransferTime(){
  double t_midstep_transfer = t_ds_; // midstep transfer time before contact transition
  return t_midstep_transfer;
}

double DCMPlannerTrajectoryManager::getFinalContactTransferTime(){
  double t_final_transfer = t_ds_ + dcm_planner_->get_settle_time(); // total time after landing the last step.
  return t_final_transfer;
}

double DCMPlannerTrajectoryManager::getSwingTime(){
  return t_ss_;
}

double DCMPlannerTrajectoryManager::getNormalForceRampUpTime(){
  return alpha_ds_*t_ds_;
}
double DCMPlannerTrajectoryManager::getNormalForceRampDownTime(){
  return (1.0 - alpha_ds_)*t_ds_;
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
void DCMPlannerTrajectoryManager::updatePreview(const int max_footsteps_to_preview){
  footstep_preview_list_.clear();
  for(int i = 0; i < max_footsteps_to_preview; i++){
    if ((i + current_footstep_index_) < footstep_list_.size()){
      footstep_preview_list_.push_back(footstep_list_[i + current_footstep_index_]);     
    }else{
      break;
    }
  }
}

bool DCMPlannerTrajectoryManager::initialize(const double t_walk_start_in,
                                             const int transfer_type_in, 
                                             const Eigen::Quaterniond & ori_start_in,
                                             const Eigen::Vector3d & dcm_pos_start_in, 
                                             const Eigen::Vector3d & dcm_vel_start_in){
  if (footstep_list_.size() == 0){
    return false;
  }

  t_walk_start_ = t_walk_start_in;

  updateStartingStance();
  left_foot_start_ = left_foot_stance_;
  right_foot_start_ = right_foot_stance_;
  updatePreview(4);

  // If preview list is empty, don't update.
  if (footstep_preview_list_.size() == 0){
    std::cout << "[DCMPlannerTrajectoryManager] ERROR. Footstep preview list is empty." << std::endl;
    return false;
  }

    // Set DCM reference
  dcm_planner_->setRobotMass(robot_->getRobotMass());
  dcm_planner_->setInitialTime(t_walk_start_);
  dcm_planner_->setInitialOri(ori_start_in);
  // Set transfer time 
  if (transfer_type_in == DCM_TRANSFER_TYPES::INITIAL){
    dcm_planner_->t_transfer = t_transfer_init_;
  } 
  else if (transfer_type_in == DCM_TRANSFER_TYPES::MIDSTEP){
    dcm_planner_->t_transfer = t_transfer_mid_;
  }

  dcm_planner_->initialize_footsteps_rvrp(footstep_preview_list_, left_foot_start_, right_foot_start_, 
                      dcm_pos_start_in, dcm_vel_start_in);


  // Initialization successful
  return true;
}


bool DCMPlannerTrajectoryManager::nextStepRobotSide(int & robot_side){
  if ( (footstep_list_.size() > 0) && (current_footstep_index_ < footstep_list_.size()) ){
    // std::cout << "hello:" << std::endl;
    // footstep_list_[current_footstep_index_].printInfo();
    robot_side = footstep_list_[current_footstep_index_].robot_side;
    return true;
  }else{
    return false;
  }

}

bool DCMPlannerTrajectoryManager::noRemainingSteps(){
  if (current_footstep_index_ >= footstep_list_.size()){
    return true;
  }else {
    return false;
  }
}

// Footstep sequence primitives -----------------------------------------------------------
// Creates footstep in place
void DCMPlannerTrajectoryManager::populateStepInPlace(const int num_steps, const int robot_side_first){
  updateStartingStance(); // Update the starting foot locations of the robot

  int robot_side = robot_side_first;
  for(int i = 0; i < num_steps; i++){
    // Add in place step and switch sides
    if (robot_side == LEFT_ROBOT_SIDE){
      footstep_list_.push_back(left_foot_stance_);
      robot_side = RIGHT_ROBOT_SIDE;
    }else{
      footstep_list_.push_back(right_foot_stance_);
      robot_side = LEFT_ROBOT_SIDE;     
    }   
  }
  // std::cout << "num_steps: " << num_steps << std::endl;
  // right_foot_stance_.printInfo();
}

// Populates the input footstep list with a predefined walking forward behavior
void DCMPlannerTrajectoryManager::populateWalkForward(const int num_steps,
                            const double nominal_step_forward_distance,
                            const double nominal_step_width_distance,
                            const double midstep_distance_multiplier){

}

// Populates the input footstep list with a predefined footstep list to turn left
void DCMPlannerTrajectoryManager::populateTurnLeft(const double turn_angle,
                             const double nominal_step_width_distance){

}

// Populates the input footstep list with a predefined footstep list to turn right
void DCMPlannerTrajectoryManager::populateTurnRight(const double turn_angle,
                              const double nominal_step_width_distance){ 
}


void DCMPlannerTrajectoryManager::paramInitialization(const YAML::Node& node){
  // void setCoMHeight(double z_vrp_in); // Sets the desired CoM Height
  // Load Custom Params ----------------------------------
  try {
    // Load DCM Parameters
    myUtils::readParameter(node,"nominal_com_height", nominal_com_height_);
    myUtils::readParameter(node,"t_additional_init_transfer", t_additional_init_transfer_);
    myUtils::readParameter(node,"t_contact_transition", t_contact_transition_);
    myUtils::readParameter(node,"t_swing", t_swing_);
    myUtils::readParameter(node,"percentage_settle", percentage_settle_);
    myUtils::readParameter(node,"alpha_ds", alpha_ds_);
  } catch(std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }
  // Convert temporal parameters to DCM parameters
  convertTemporalParamsToDCMParams();

  // Set DCM parameters
  dcm_planner_->t_transfer = t_transfer_init_; // Time varying after every step
  dcm_planner_->t_ds = t_ds_;
  dcm_planner_->t_ss = t_ss_;
  dcm_planner_->percentage_settle = percentage_settle_;
  dcm_planner_->alpha_ds = alpha_ds_;
}



