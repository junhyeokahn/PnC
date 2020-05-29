#include <PnC/ValkyriePnC/TrajectoryManagers/FootSE3TrajectoryManager.hpp>

FootSE3TrajectoryManager::FootSE3TrajectoryManager(Task* _foot_pos_task, Task* _foot_ori_task, RobotSystem* _robot): TrajectoryManagerBase(_robot){
    myUtils::pretty_constructor(2, "TrajectoryManager: FootSE3");
	// Set Linear and Orientation Foot task
	foot_pos_task_ = _foot_pos_task;
	foot_ori_task_ = _foot_ori_task;

	// Assume that both tasks use the same link id.
	link_idx_ = static_cast<BasicTask*>(foot_pos_task_)->getLinkID();

  // Initialize member variables
  foot_pos_des_.setZero(); 
  foot_vel_des_.setZero();
  foot_acc_des_.setZero();

  foot_quat_des_.setIdentity();
  foot_ori_des_ = Eigen::VectorXd::Zero(4);
  foot_ang_vel_des_.setZero();
  foot_ang_acc_des_.setZero();

  swing_height_ = 0.04; // 4cm default
}


FootSE3TrajectoryManager::~FootSE3TrajectoryManager(){
}

void FootSE3TrajectoryManager::paramInitialization(const YAML::Node& node){	
    try {
        myUtils::readParameter(node,"swing_height", swing_height_);

     } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}

void FootSE3TrajectoryManager::useCurrent(){
  // Update desired to use current foot pose
  foot_pos_des_ = robot_->getBodyNodeCoMIsometry(link_idx_).translation();
  foot_quat_des_ = robot_->getBodyNodeCoMIsometry(link_idx_).linear();
  convertQuatDesToOriDes();
  updateDesired();
}

void FootSE3TrajectoryManager::convertQuatDesToOriDes(){
  foot_ori_des_[0] = foot_quat_des_.w();
  foot_ori_des_[1] = foot_quat_des_.x();
  foot_ori_des_[2] = foot_quat_des_.y();
  foot_ori_des_[3] = foot_quat_des_.z();
}

void FootSE3TrajectoryManager::updateDesired(){
  Eigen::Vector3d cur_foot_pos = robot_->getBodyNodeCoMIsometry(link_idx_).translation();

  // myUtils::pretty_print(cur_foot_pos, std::cout, "cur_foot_pos");
  // myUtils::pretty_print(foot_pos_des_, std::cout, "foot_pos_des_");

  foot_pos_task_->updateDesired(foot_pos_des_, foot_vel_des_, foot_acc_des_);
  foot_ori_task_->updateDesired(foot_ori_des_, foot_ang_vel_des_, foot_ang_acc_des_);
}

// Initialize the swing foot trajectory 
void FootSE3TrajectoryManager::initializeSwingFootTrajectory(const double _start_time, const double _swing_duration, const Footstep & _landing_foot){
  // Copy and initialize variables
  swing_start_time_ = _start_time;
  swing_duration_ = _swing_duration;  
  swing_land_foot_ = _landing_foot;

  // Initialize swing foot starting pose
  Eigen::Vector3d start_foot_pos = robot_->getBodyNodeCoMIsometry(link_idx_).translation();
  Eigen::Quaterniond start_foot_ori(robot_->getBodyNodeCoMIsometry(link_idx_).linear());

  swing_init_foot_.setPosOriSide(start_foot_pos,
                                 start_foot_ori, 
                                 swing_land_foot_.robot_side);               
 
  // Compute where the foot will be in the middle of the trajectory
  swing_midfoot_.computeMidfeet(swing_init_foot_, swing_land_foot_, swing_midfoot_);

  // Compute midfeet boundary conditions
  // Linear velocity at the middle of the swing is the total swing travel over swing time  
  Eigen::Vector3d mid_swing_local_foot_pos(0, 0, swing_height_);
  Eigen::Vector3d mid_swing_position = swing_midfoot_.position + swing_midfoot_.R_ori*mid_swing_local_foot_pos;
  Eigen::Vector3d mid_swing_velocity = (swing_land_foot_.position - swing_init_foot_.position)/swing_duration_;

  myUtils::pretty_print(mid_swing_position, std::cout, "mid_swing_position");
  myUtils::pretty_print(mid_swing_velocity, std::cout, "mid_swing_velocity");

  // Construct Position trajectories  
  pos_traj_init_to_mid_.initialize(swing_init_foot_.position, Eigen::Vector3d::Zero(3), 
                                   mid_swing_position, mid_swing_velocity);
  pos_traj_mid_to_end_.initialize(mid_swing_position, mid_swing_velocity,
                                  swing_land_foot_.position, Eigen::Vector3d::Zero(3));

  // Construct Quaternion trajectory
  Eigen::Vector3d ang_vel_start; ang_vel_start.setZero();
  Eigen::Vector3d ang_vel_end; ang_vel_end.setZero();

  quat_hermite_curve_.initialize(swing_init_foot_.orientation, ang_vel_start,
                                 swing_land_foot_.orientation, ang_vel_end);
}

// Computes the swing foot
void FootSE3TrajectoryManager::computeSwingFoot(const double current_time){
      // Compute progression variable
      double s = (current_time - swing_start_time_)/swing_duration_;

      // Get foot position and its derivatives
      if (s <= 0.5){ // 0.0 <= s < 0.5 use the first trajectory
        // scale back to 1.0
        s = 2.0*s;
        foot_pos_des_ = pos_traj_init_to_mid_.evaluate(s);
        foot_vel_des_ = pos_traj_init_to_mid_.evaluateFirstDerivative(s);
        foot_acc_des_ = pos_traj_init_to_mid_.evaluateSecondDerivative(s);
      }else{         // 0.5 <= s < 1.0 use the second trajectory
        // scale back to 1.0 after the offset
        s = 2.0*(s - 0.5);
        foot_pos_des_ = pos_traj_mid_to_end_.evaluate(s);
        foot_vel_des_ = pos_traj_mid_to_end_.evaluateFirstDerivative(s);
        foot_acc_des_ = pos_traj_mid_to_end_.evaluateSecondDerivative(s);
      }

      // Get foot orientation and its derivatives
      s = (current_time - swing_start_time_)/swing_duration_;
      quat_hermite_curve_.evaluate(s, foot_quat_des_);
      quat_hermite_curve_.getAngularVelocity(s, foot_ang_vel_des_);
      quat_hermite_curve_.getAngularAcceleration(s, foot_ang_acc_des_);
      convertQuatDesToOriDes();

      // myUtils::pretty_print(foot_pos_des_, std::cout, "foot_pos_des_");
      // myUtils::pretty_print(foot_ori_des_, std::cout, "foot_ori_des_");

}

void FootSE3TrajectoryManager::updateSwingFootDesired(const double current_time){
  computeSwingFoot(current_time);
  updateDesired();
}
