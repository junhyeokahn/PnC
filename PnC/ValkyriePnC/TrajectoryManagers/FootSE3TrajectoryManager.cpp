#include <PnC/ValkyriePnC/TrajectoryManagers/FootSE3TrajectoryManager.hpp>

FootSE3TrajectoryManager::FootSE3TrajectoryManager(Task* _foot_pos_task, Task* _foot_ori_task, RobotSystem* _robot): TrajectoryManagerBase(_robot){
    myUtils::pretty_constructor(2, "TrajectoryManager: FootSE3");
	// Set Linear and Orientation Foot task
	foot_pos_task_ = _foot_pos_task;
	foot_ori_task_ = _foot_ori_task;

	// Assume that both tasks use the same link id.
	link_idx_ = static_cast<BasicTask*>(foot_pos_task_)->getLinkID();

  // Initialize member variables
  foot_pos_des_ = Eigen::VectorXd::Zero(3); 
  foot_vel_des_ = Eigen::VectorXd::Zero(3);
  foot_acc_des_ = Eigen::VectorXd::Zero(3);

  foot_quat_des_.setIdentity();
  foot_ori_des_ = Eigen::VectorXd::Zero(4);
  foot_ang_vel_des_ = Eigen::VectorXd::Zero(3);
  foot_ang_acc_des_ = Eigen::VectorXd::Zero(3);

}


FootSE3TrajectoryManager::~FootSE3TrajectoryManager(){
}

void FootSE3TrajectoryManager::paramInitialization(const YAML::Node& node){	
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
  foot_pos_task_->updateDesired(foot_pos_des_, foot_vel_des_, foot_acc_des_);
  foot_ori_task_->updateDesired(foot_ori_des_, foot_ang_vel_des_, foot_ang_acc_des_);
}

// Initialize the swing foot trajectory 
void FootSE3TrajectoryManager::initializeSwingFootTrajectory(const double _start_time, const double _swing_duration, const Footstep & _landing_foot){
  swing_start_time_ = _start_time;
  swing_duration_ = _swing_duration;  
}

// Computes the swing foot
void FootSE3TrajectoryManager::computeSwingFoot(const double current_time){

}
