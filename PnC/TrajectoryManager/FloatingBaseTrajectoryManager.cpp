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

  amp = Eigen::VectorXd::Zero(3);
  freq = Eigen::VectorXd::Zero(3);
  mid_point = Eigen::VectorXd::Zero(3);
  is_swaying = false;
  is_sinusoid = false;
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
  // base_ori_quat_des_ =
  // Eigen::Quaternion<double>(robot_->getBodyNodeIsometry(base_id_).linear());
  base_ori_quat_des_ = Eigen::Quaternion<double>::Identity();
  ini_base_quat_ << base_ori_quat_des_.w(), base_ori_quat_des_.x(),
      base_ori_quat_des_.y(), base_ori_quat_des_.z();
  target_com_pos_ = _target_com_pos;
}

void FloatingBaseTrajectoryManager::initializeCoMSwaying(double _start_time,
                                                         double _duration,
                                                         Eigen::VectorXd _dis) {
  is_swaying = true;
  start_time_ = _start_time;
  duration_ = _duration;
  // ini_com_pos_ = ((Eigen::VectorXd)robot_->getCoMPosition());
  ini_com_pos_ = target_com_pos_;
  // base_ori_quat_des_ =
  // Eigen::Quaternion<double>(robot_->getBodyNodeIsometry(base_id_).linear());
  base_ori_quat_des_ = Eigen::Quaternion<double>::Identity();
  ini_base_quat_ << base_ori_quat_des_.w(), base_ori_quat_des_.x(),
      base_ori_quat_des_.y(), base_ori_quat_des_.z();
  target_com_pos_ = ini_com_pos_ + _dis;
}

void FloatingBaseTrajectoryManager::initializeCoMSinusoid(double _start_time,
                                                          double _amp,
                                                          double _freq) {
  is_sinusoid = true;
  start_time_ = _start_time;
  mid_point = target_com_pos_;
  amp << 0., _amp, 0;
  freq << 0., _freq, 0;
  base_ori_quat_des_ = Eigen::Quaternion<double>::Identity();
  ini_base_quat_ << base_ori_quat_des_.w(), base_ori_quat_des_.x(),
      base_ori_quat_des_.y(), base_ori_quat_des_.z();
}

void FloatingBaseTrajectoryManager::updateFloatingBaseDesired(
    const double current_time) {
  if (is_sinusoid) {
    myUtils::getSinusoidTrajectory(start_time_, mid_point, amp, freq,
                                   current_time, com_pos_des_, com_vel_des_,
                                   com_acc_des_);
  } else {
    for (int i = 0; i < 3; ++i) {
      com_pos_des_[i] =
          myUtils::smooth_changing(ini_com_pos_[i], target_com_pos_[i],
                                   duration_, current_time - start_time_);
      com_vel_des_[i] =
          myUtils::smooth_changing_vel(ini_com_pos_[i], target_com_pos_[i],
                                       duration_, current_time - start_time_);
      com_acc_des_[i] =
          myUtils::smooth_changing_acc(ini_com_pos_[i], target_com_pos_[i],
                                       duration_, current_time - start_time_);
    }
  }

  base_ori_des_ = ini_base_quat_;
  base_ang_vel_des_ = Eigen::VectorXd::Zero(3);
  base_ang_acc_des_ = Eigen::VectorXd::Zero(3);

  updateDesired();
}
