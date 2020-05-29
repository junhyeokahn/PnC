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

  // Nominal walking parameters  
  nominal_footwidth_ = 0.27;
  nominal_forward_step_ = 0.25;
  nominal_backward_step_ = -0.25;
  nominal_turn_radians_ = M_PI/4.0;
  nominal_strafe_distance_ = 0.125;   

  // First step before alternating
  robot_side_first_ = RIGHT_ROBOT_SIDE;

  convertTemporalParamsToDCMParams();
}

DCMPlannerTrajectoryManager::~DCMPlannerTrajectoryManager(){
}

void DCMPlannerTrajectoryManager::setCoMandPelvisTasks(Task* _com_task, Task* _pelvis_ori_task_){
  com_task_ = _com_task;
  pelvis_ori_task_ = _pelvis_ori_task_;
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

    mid_foot_stance_.computeMidfeet(left_foot_stance_, right_foot_stance_, mid_foot_stance_);
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

  // std::cout << "[DCMPlannerTrajectoryManager]" << std::endl;
  // std::cout << "  current_footstep_index = " << current_footstep_index_ << std::endl;
  // std::cout << "  preview size = " << footstep_preview_list_.size() << std::endl;

  // If preview list is empty, don't update.
  if (footstep_preview_list_.size() == 0){
    std::cout << "[DCMPlannerTrajectoryManager] ERROR. Footstep preview list is empty." << std::endl;
    return false;
  }

    // Set DCM reference
  dcm_planner_->setRobotMass(robot_->getRobotMass());
  dcm_planner_->setCoMHeight(nominal_com_height_);
  dcm_planner_->setInitialTime(t_walk_start_);
  dcm_planner_->setInitialOri(ori_start_in);
  
  std::cout << "ori_start_in:" << ori_start_in.w() << ", "
                               << ori_start_in.x() << ", "
                               << ori_start_in.y() << ", "
                               << ori_start_in.z() << ", " << std::endl;
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

void DCMPlannerTrajectoryManager::updateDCMTasksDesired(double current_time){
  // Initialize containers
  Eigen::Vector3d des_com_pos, des_com_vel, des_com_acc; 
  des_com_pos.setZero(); des_com_vel.setZero(); des_com_acc.setZero();

  Eigen::Quaterniond des_quat; des_quat.setIdentity(); 
  Eigen::Vector3d des_ang_vel, des_ang_acc;
  des_ang_vel.setZero(); des_ang_acc.setZero();

  dcm_planner_->get_ref_com(current_time, des_com_pos);
  dcm_planner_->get_ref_com_vel(current_time, des_com_vel);
  dcm_planner_->get_ref_ori_ang_vel_acc(current_time, des_quat,
                                                      des_ang_vel,
                                                      des_ang_acc);

  Eigen::VectorXd des_quat_vec = Eigen::VectorXd::Zero(4);
  des_quat_vec << des_quat.w(),
                  des_quat.x(), 
                  des_quat.y(),
                  des_quat.z();

  // std::cout << "current_time: " << current_time << std::endl;
  // myUtils::pretty_print(des_com_pos, std::cout, "des_com_pos");
  // myUtils::pretty_print(des_quat_vec, std::cout, "des_quat_vec");


  com_task_->updateDesired(des_com_pos, des_com_vel, des_com_acc);
  pelvis_ori_task_->updateDesired(des_quat_vec, des_ang_vel, des_ang_acc);
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


void DCMPlannerTrajectoryManager::alternateLeg(){
  if (robot_side_first_ == LEFT_ROBOT_SIDE){
    robot_side_first_ = RIGHT_ROBOT_SIDE;
  }else{
    robot_side_first_ = LEFT_ROBOT_SIDE;
  }  
}

void DCMPlannerTrajectoryManager::resetIndexAndClearFootsteps(){
  // Reset index and footstep list
  resetStepIndex();
  footstep_list_.clear();     
}

void DCMPlannerTrajectoryManager::walkInPlace(){
  resetIndexAndClearFootsteps();
  populateStepInPlace(2, robot_side_first_);
  alternateLeg();
}
void DCMPlannerTrajectoryManager::walkForward(){
  resetIndexAndClearFootsteps();
  populateWalkForward(2, nominal_forward_step_);
  alternateLeg();
}
void DCMPlannerTrajectoryManager::walkBackward(){
  resetIndexAndClearFootsteps();
  populateWalkForward(2, nominal_backward_step_);
  alternateLeg();
}
void DCMPlannerTrajectoryManager::strafeLeft(){
  resetIndexAndClearFootsteps();
  populateStrafe(nominal_strafe_distance_, 1);
}
void DCMPlannerTrajectoryManager::strafeRight(){
  resetIndexAndClearFootsteps();
  populateStrafe(-nominal_strafe_distance_, 1);
}
void DCMPlannerTrajectoryManager::turnLeft(){
  resetIndexAndClearFootsteps();
  populateRotateTurn(nominal_turn_radians_, 1);
}
void DCMPlannerTrajectoryManager::turnRight(){
  resetIndexAndClearFootsteps();
  populateRotateTurn(-nominal_turn_radians_, 1);
}


// Footstep sequence primitives -----------------------------------------------------------
// Creates footstep in place
void DCMPlannerTrajectoryManager::populateStepInPlace(const int num_steps, const int robot_side_first){
  updateStartingStance(); // Update the starting foot locations of the robot

  Footstep left_footstep = left_foot_stance_;
  Footstep right_footstep = right_foot_stance_;
  Footstep mid_footstep = mid_foot_stance_; 

  int robot_side = robot_side_first;
  for(int i = 0; i < num_steps; i++){
    // Square feet and switch sides
    if (robot_side == LEFT_ROBOT_SIDE){
      left_footstep.setPosOri(mid_footstep.position + mid_footstep.R_ori*Eigen::Vector3d(0, nominal_footwidth_/2.0, 0.0),
                              mid_footstep.orientation);
      footstep_list_.push_back(left_footstep);
      robot_side = RIGHT_ROBOT_SIDE;
    }else{
      right_footstep.setPosOri(mid_footstep.position + mid_footstep.R_ori*Eigen::Vector3d(0, -nominal_footwidth_/2.0, 0.0),
                              mid_footstep.orientation);
      footstep_list_.push_back(right_footstep);
      robot_side = LEFT_ROBOT_SIDE;     
    }   
  }
}

// Populates the input footstep list with a predefined walking forward behavior
void DCMPlannerTrajectoryManager::populateWalkForward(const int num_steps,
                         const double forward_distance){

  updateStartingStance(); // Update the starting foot locations of the robot

  Footstep new_footstep;
  Footstep mid_footstep = mid_foot_stance_; 

  int robot_side = LEFT_ROBOT_SIDE;
  for(int i = 0; i < num_steps; i++){
    if (robot_side == LEFT_ROBOT_SIDE){
      Eigen::Vector3d translate((i+1)*forward_distance, nominal_footwidth_/2.0, 0);
      new_footstep.setPosOriSide(mid_footstep.position + mid_footstep.R_ori*translate,
                                 mid_footstep.orientation, LEFT_ROBOT_SIDE);
      robot_side = RIGHT_ROBOT_SIDE;
    }else{
      Eigen::Vector3d translate((i+1)*forward_distance, -nominal_footwidth_/2.0, 0);      
      new_footstep.setPosOriSide(mid_footstep.position + mid_footstep.R_ori*translate,
                                 mid_footstep.orientation, RIGHT_ROBOT_SIDE);
      robot_side = LEFT_ROBOT_SIDE;
    }
    footstep_list_.push_back(new_footstep);
  }

  // Add additional step forward to square the feet.
  if (robot_side == LEFT_ROBOT_SIDE){
    Eigen::Vector3d translate(num_steps*forward_distance, nominal_footwidth_/2.0, 0);
    new_footstep.setPosOriSide(mid_footstep.position + mid_footstep.R_ori*translate,
                           mid_footstep.orientation, LEFT_ROBOT_SIDE);
  }else{
    Eigen::Vector3d translate(num_steps*forward_distance, -nominal_footwidth_/2.0, 0);
    new_footstep.setPosOriSide(mid_footstep.position + mid_footstep.R_ori*translate,
                               mid_footstep.orientation, RIGHT_ROBOT_SIDE);
  }
  footstep_list_.push_back(new_footstep);

}

// Take two steps to rotate at the specified radians. Repeat num_times
void DCMPlannerTrajectoryManager::populateRotateTurn(const double turn_radians_per_step, const int num_times){

  updateStartingStance(); // Update the starting foot locations of the robot

  Eigen::Quaterniond foot_rotate( Eigen::AngleAxisd(turn_radians_per_step, Eigen::Vector3d::UnitZ()) );

  Footstep left_footstep, right_footstep;
  Footstep mid_footstep = mid_foot_stance_; 

  Footstep mid_footstep_rotated = mid_footstep;
  for(int i = 0; i < num_times; i++){

    mid_footstep_rotated.setPosOri(mid_footstep.position, foot_rotate*mid_footstep.orientation);

    left_footstep.setPosOriSide(mid_footstep_rotated.position + mid_footstep_rotated.R_ori*Eigen::Vector3d(0, nominal_footwidth_/2.0, 0), 
                                mid_footstep_rotated.orientation, LEFT_ROBOT_SIDE);
    right_footstep.setPosOriSide(mid_footstep_rotated.position + mid_footstep_rotated.R_ori*Eigen::Vector3d(0, -nominal_footwidth_/2.0, 0), 
                                mid_footstep_rotated.orientation, RIGHT_ROBOT_SIDE);

    if (turn_radians_per_step > 0){
      footstep_list_.push_back(left_footstep);
      footstep_list_.push_back(right_footstep);
    }else{
      footstep_list_.push_back(right_footstep);    
      footstep_list_.push_back(left_footstep);
    }
    mid_footstep = mid_footstep_rotated;
  }

}

// Take two steps to strafe at the specified distance. Repeat num_times.
void DCMPlannerTrajectoryManager::populateStrafe(const double strafe_distance, const int num_times){
  updateStartingStance(); // Update the starting foot locations of the robot

  // Strafe 
  Footstep left_footstep, right_footstep;
  Footstep mid_footstep = mid_foot_stance_; 
  Footstep mid_footstep_translated = mid_footstep;

  for(int i = 0; i < num_times; i++){
    mid_footstep_translated.setPosOri(mid_footstep.position + mid_footstep.R_ori*Eigen::Vector3d(0.0, strafe_distance, 0.0), mid_footstep.orientation);

    left_footstep.setPosOriSide(mid_footstep_translated.position + mid_footstep_translated.R_ori*Eigen::Vector3d(0, nominal_footwidth_/2.0, 0), 
                                mid_footstep_translated.orientation, LEFT_ROBOT_SIDE);
    right_footstep.setPosOriSide(mid_footstep_translated.position + mid_footstep_translated.R_ori*Eigen::Vector3d(0, -nominal_footwidth_/2.0, 0), 
                                mid_footstep_translated.orientation, RIGHT_ROBOT_SIDE);

    if (strafe_distance > 0){
      // Left strafe
      footstep_list_.push_back(left_footstep);
      footstep_list_.push_back(right_footstep);
    }else{
      // Right strafe
      footstep_list_.push_back(right_footstep);    
      footstep_list_.push_back(left_footstep);
    }
    mid_footstep = mid_footstep_translated;    
  }  
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

    // Load Walking Primitives Parameters
    myUtils::readParameter(node, "nominal_footwidth", nominal_footwidth_);
    myUtils::readParameter(node, "nominal_forward_step", nominal_forward_step_);
    myUtils::readParameter(node, "nominal_backward_step", nominal_backward_step_);
    myUtils::readParameter(node, "nominal_turn_radians", nominal_turn_radians_);
    myUtils::readParameter(node, "nominal_strafe_distance", nominal_strafe_distance_);       


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



