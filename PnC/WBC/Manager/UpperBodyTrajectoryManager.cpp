#include <PnC/WBC/Manager/UpperBodyTrajectoryManager.hpp>

UpperBodyTrajectoryManager::UpperBodyTrajectoryManager(Task *_upper_body_task,
                                                       RobotSystem *_robot) {
  upper_body_task_ = _upper_body_task;
  robot_ = _robot;
  target_ids_ = upper_body_task_->target_ids;
}

UpperBodyTrajectoryManager::~UpperBodyTrajectoryManager() {}

void UpperBodyTrajectoryManager::UseNominalUpperBodyJointPos(
    std::map<std::string, double> _nominal_joint_pos) {

  Eigen::VectorXd joint_pos_des = Eigen::VectorXd::Zero(target_ids_.size());
  Eigen::VectorXd joint_vel_des = Eigen::VectorXd::Zero(target_ids_.size());
  Eigen::VectorXd joint_acc_des = Eigen::VectorXd::Zero(target_ids_.size());

  for (int i = 0; i < target_ids_.size(); ++i)
    joint_pos_des[i] = _nominal_joint_pos[target_ids_[i]];

  upper_body_task_->update_desired(joint_pos_des, joint_vel_des, joint_acc_des);
}
