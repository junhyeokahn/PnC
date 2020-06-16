#include <PnC/TrajectoryManager/JointTrajectoryManager.hpp>

JointTrajectoryManager::JointTrajectoryManager(Task* _joint_task,
                                               RobotSystem* _robot)
    : TrajectoryManagerBase(_robot) {
  myUtils::pretty_constructor(2, "TrajectoryManager: JointPos");
  joint_task_ = _joint_task;
  joint_pos_des_ = Eigen::VectorXd::Zero(joint_task_->getDim());
  joint_vel_des_ = Eigen::VectorXd::Zero(joint_task_->getDim());
  joint_acc_des_ = Eigen::VectorXd::Zero(joint_task_->getDim());
}

void JointTrajectoryManager::updateDesired() {
  joint_task_->updateDesired(joint_pos_des_, joint_vel_des_, joint_acc_des_);
}

void JointTrajectoryManager::initializeJointTrajectory(
    const double _start_time, const double _duration,
    const Eigen::VectorXd& _target_jpos) {
  start_time_ = _start_time;
  duration_ = _duration;
  ini_jpos_ = robot_->getQ().tail(joint_task_->getDim());
  target_jpos_ = _target_jpos;
}

void JointTrajectoryManager::useCurrent() {
  joint_pos_des_ = robot_->getQ().tail(joint_task_->getDim());
  joint_vel_des_ = robot_->getQ().tail(joint_task_->getDim());
  joint_acc_des_ = Eigen::VectorXd::Zero(joint_task_->getDim());
  updateDesired();
}

void JointTrajectoryManager::updateJointDesired(const double current_time) {
  for (int i = 0; i < joint_task_->getDim(); ++i) {
    joint_pos_des_[i] = myUtils::smooth_changing(ini_jpos_[i], target_jpos_[i],
                                                 duration_, current_time);
    joint_vel_des_[i] = myUtils::smooth_changing_vel(
        ini_jpos_[i], target_jpos_[i], duration_, current_time);
    joint_acc_des_[i] = myUtils::smooth_changing_acc(
        ini_jpos_[i], target_jpos_[i], duration_, current_time);
  }
  updateDesired();
}
