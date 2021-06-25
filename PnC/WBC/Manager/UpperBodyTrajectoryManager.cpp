#include <PnC/WBC/Manager/UpperBodyTrajectoryManager.hpp>

UpperBodyTrajectoryManager::UpperBodyTrajectoryManager(Task *_upper_body_task,
                                                       RobotSystem *_robot) {
  upper_body_task_ = _upper_body_task;
  robot_ = _robot;
}

UpperBodyTrajectoryManager::~UpperBodyTrajectoryManager() {}

void UpperBodyTrajectoryManager::UseNominalUpperBodyJointPos(
    std::map<std::string, double> _nominal_joint_pos) {

  Eigen::VectorXd joint_pos_des =
      Eigen::VectorXd::Zero(_nominal_joint_pos.size());
  Eigen::VectorXd joint_vel_des =
      Eigen::VectorXd::Zero(_nominal_joint_pos.size());
  Eigen::VectorXd joint_acc_des =
      Eigen::VectorXd::Zero(_nominal_joint_pos.size());

  int i(0);
  for (std::map<std::string, double>::iterator it = _nominal_joint_pos.begin();
       it != _nominal_joint_pos.end(); ++it) {
    joint_pos_des[i] = it->second;
    i += 1;
  }

  upper_body_task_->update_desired(joint_pos_des, joint_vel_des, joint_acc_des);
}
