#include <PnC/TrajectoryManager/FloatingBaseTrajectoryManager.hpp>

FloatingBaseTrajectoryManager::FloatingBaseTrajectoryManager(
    Task* _com_task, Task* _base_ori_task, RobotSystem* _robot)
    : TrajectoryManagerBase(_robot) {
  myUtils::pretty_constructor(2, "TrajectoryManager: Floating Base");

  com_task_ = _com_task;
  base_ori_task_ = _base_ori_task;
  base_id_ = base_ori_task_->getLinkID();

  com_pos_des_ = Eigen::VectorXd::Zero(3);
  com_vel_des_ = Eigen::VectorXd::Zero(3);
  com_acc_des_ = Eigen::VectorXd::Zero(3);

  dcm_pos_des_ = Eigen::VectorXd::Zero(3);
  dcm_vel_des_ = Eigen::VectorXd::Zero(3);
  dcm_acc_des_ = Eigen::VectorXd::Zero(3);

  base_ori_des_ = Eigen::VectorXd::Zero(4);
  base_ang_vel_des_ = Eigen::VectorXd::Zero(3);
  base_ang_acc_des_ = Eigen::VectorXd::Zero(3);

  ini_com_pos_ = Eigen::VectorXd::Zero(3);
  ini_base_quat_ = Eigen::VectorXd::Zero(4);
  target_com_pos_ = Eigen::VectorXd::Zero(3);
}

void FloatingBaseTrajectoryManager::updateDesired() {
  com_task_->updateDesired(com_pos_des_, com_vel_des_, com_acc_des_);
  base_ori_task_->updateDesired(base_ori_des_, base_ang_vel_des_,
                                base_ang_acc_des_);
  // TEST
  double dcm_omega = sqrt(9.81 / robot_->getCoMPosition()[2]);
  dcm_pos_des_ = com_pos_des_ + com_vel_des_ / dcm_omega;
  dcm_vel_des_ = com_vel_des_ + com_acc_des_ / dcm_omega;
  dcm_pos_des_[2] = com_pos_des_[2];
  dcm_vel_des_[2] = com_vel_des_[2];
  dcm_acc_des_.setZero();
  // com_task_->updateDesired(dcm_pos_des_, dcm_vel_des_, dcm_acc_des_);
  // TEST
}

void FloatingBaseTrajectoryManager::initializeFloatingBaseTrajectory(
    const double _start_time, const double _duration,
    const Eigen::VectorXd& _target_com_pos) {
  start_time_ = _start_time;
  duration_ = _duration;
  ini_com_pos_ = ((Eigen::VectorXd)robot_->getCoMPosition());
  base_ori_quat_des_ =
      Eigen::Quaternion<double>(robot_->getBodyNodeIsometry(base_id_).linear());
  ini_base_quat_ << base_ori_quat_des_.w(), base_ori_quat_des_.x(),
      base_ori_quat_des_.y(), base_ori_quat_des_.z();
  target_com_pos_ = _target_com_pos;
}

void FloatingBaseTrajectoryManager::updateFloatingBaseDesired(
    const double current_time) {
  for (int i = 0; i < 3; ++i) {
    com_pos_des_[i] = myUtils::smooth_changing(
        ini_com_pos_[i], target_com_pos_[i], duration_, current_time);
    com_vel_des_[i] = myUtils::smooth_changing_vel(
        ini_com_pos_[i], target_com_pos_[i], duration_, current_time);
    com_acc_des_[i] = myUtils::smooth_changing_acc(
        ini_com_pos_[i], target_com_pos_[i], duration_, current_time);
  }

  base_ori_des_ = ini_base_quat_;
  base_ang_vel_des_ = Eigen::VectorXd::Zero(3);
  base_ang_acc_des_ = Eigen::VectorXd::Zero(3);

  updateDesired();
}
