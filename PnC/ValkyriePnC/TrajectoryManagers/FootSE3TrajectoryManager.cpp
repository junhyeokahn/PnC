#include <PnC/ValkyriePnC/TrajectoryManagers/FootSE3TrajectoryManager.hpp>

FootSE3TrajectoryManager::FootSE3TrajectoryManager(Task* _foot_pos_task, Task* _foot_ori_task, RobotSystem* _robot): TrajectoryManagerBase(_robot){
    myUtils::pretty_constructor(2, "TrajectoryManager: FootSE3");
	// Set Linear and Orientation Foot task
	foot_pos_task_ = _foot_pos_task;
	foot_ori_task_ = _foot_ori_task;

	// Assume that both tasks use the same link id.
	link_idx_ = static_cast<BasicTask*>(foot_pos_task_)->getLinkID();
}


FootSE3TrajectoryManager::~FootSE3TrajectoryManager(){
}

void FootSE3TrajectoryManager::paramInitialization(const YAML::Node& node){	
}

void FootSE3TrajectoryManager::useCurrent(){
  Eigen::VectorXd foot_pos_des(3); foot_pos_des.setZero();
  Eigen::VectorXd foot_vel_des(3); foot_vel_des.setZero();    
  Eigen::VectorXd foot_acc_des(3); foot_acc_des.setZero();    

  Eigen::VectorXd foot_ori_des(4); foot_ori_des.setZero();
  Eigen::VectorXd foot_ang_vel_des(3); foot_ang_vel_des.setZero();    
  Eigen::VectorXd foot_ang_acc_des(3); foot_ang_acc_des.setZero();

  // Set Foot Task
  foot_pos_des = robot_->getBodyNodeCoMIsometry(link_idx_).translation();
  Eigen::Quaternion<double> rfoot_ori_act(robot_->getBodyNodeCoMIsometry(link_idx_).linear());
  foot_ori_des[0] = rfoot_ori_act.w();
  foot_ori_des[1] = rfoot_ori_act.x();
  foot_ori_des[2] = rfoot_ori_act.y();
  foot_ori_des[3] = rfoot_ori_act.z();

  foot_pos_task_->updateDesired(foot_pos_des, foot_vel_des, foot_acc_des);
  foot_ori_task_->updateDesired(foot_ori_des, foot_ang_vel_des, foot_ang_acc_des);
}