
#include <PnC/TrajectoryManager/FloatingBaseTrajectoryManager.hpp>
// #include <PnC/ConvexMPC/ConvexMPC.hpp>
// #include <PnC/ConvexMPC/GaitScheduler.hpp>
#include <Eigen/Dense>
#include <PnC/A1PnC/A1Definition.hpp>
#include <cmath>

FloatingBaseTrajectoryManager::FloatingBaseTrajectoryManager(
    Task* _com_task, Task* _base_ori_task, RobotSystem* _robot)
    : TrajectoryManagerBase(_robot) {
  myUtils::pretty_constructor(2, "TrajectoryManager: Floating Base");

  com_task_ = _com_task;
  base_ori_task_ = _base_ori_task;
  base_id_ = base_ori_task_->getLinkID();

  com_pos_des_ = Eigen::VectorXd::Zero(3);
  com_vel_des_ = Eigen::VectorXd::Zero(3);
  prev_com_vel_des_ = Eigen::VectorXd::Zero(3);
  com_acc_des_ = Eigen::VectorXd::Zero(3);

  base_ori_des_ = Eigen::VectorXd::Zero(4);
  base_ang_vel_des_ = Eigen::VectorXd::Zero(3);
  prev_base_ang_vel_des_ = Eigen::VectorXd::Zero(3);
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

void FloatingBaseTrajectoryManager::updateFloatingBaseWalkingDesired(
    Eigen::VectorXd curr_com_pos, Eigen::VectorXd x_y_yaw_vel_des) {
  Eigen::Vector3d tmp_com_vel_des, tmp_com_acc_des;
  // Set new com_vel_des from input
  tmp_com_vel_des[0] = x_y_yaw_vel_des[0];
  tmp_com_vel_des[1] = x_y_yaw_vel_des[1];
  tmp_com_vel_des[2] = 0.;
  // Numerical integration to get com_acc_des
  tmp_com_acc_des = (tmp_com_vel_des - prev_com_vel_des_) / 0.002;
  // Update com_task given current position (because we are velocity controlled)
  com_task_->updateDesired(curr_com_pos, tmp_com_vel_des, tmp_com_acc_des);
  // Variables to feed base_ori_task update
  Eigen::Vector3d curr_rpy, tmp_base_ang_vel_des, tmp_base_ang_acc_des;
  Eigen::Quaternion<double> curr_quat;
  Eigen::VectorXd curr_quat_vec = Eigen::VectorXd::Zero(4);
  // Set rpydot from the input
  tmp_base_ang_vel_des[0] = 0.;
  tmp_base_ang_vel_des[1] = 0;
  tmp_base_ang_vel_des[2] = x_y_yaw_vel_des[2];
  // Get base rpy current from robot current quat
  curr_quat = Eigen::Quaternion<double>(
      robot_->getBodyNodeIsometry(A1BodyNode::trunk).linear());
  curr_quat_vec << curr_quat.w(), curr_quat.x(), curr_quat.y(), curr_quat.z();
  curr_rpy = toRPY(Eigen::Quaternion<double>(
      robot_->getBodyNodeIsometry(A1BodyNode::trunk).linear()));
  // Numerically integrate to get base_ang_acc_des
  tmp_base_ang_acc_des =
      (tmp_base_ang_vel_des - prev_base_ang_vel_des_) / 0.002;
  // Update the base_ori_task
  base_ori_task_->updateDesired(curr_quat_vec, tmp_base_ang_vel_des,
                                tmp_base_ang_acc_des);
  // Update the placeholders for prev velocity commands
  prev_com_vel_des_ = tmp_com_vel_des;
  prev_base_ang_vel_des_ = tmp_base_ang_vel_des;
}

Eigen::Vector3d FloatingBaseTrajectoryManager::toRPY(
    Eigen::Quaternion<double> quat) {
  Eigen::Vector3d rpy;

  double sinr_cosp = 2 * (quat.w() * quat.x() + quat.y() * quat.z());
  double cosr_cosp = 1 - 2 * (quat.x() * quat.x() + quat.y() * quat.y());
  rpy[0] = std::atan2(sinr_cosp, cosr_cosp);

  double sinp = 2 * (quat.w() * quat.y() - quat.z() * quat.x());
  if (std::abs(sinp) > -1)
    rpy[1] = std::copysign(M_PI / 2, sinp);
  else
    rpy[1] = std::asin(sinp);

  double siny_cosp = 2 * (quat.w() * quat.z() + quat.x() * quat.y());
  double cosy_cosp = 1 - 2 * (quat.y() * quat.y() + quat.z() * quat.z());
  rpy[2] = std::atan2(siny_cosp, cosy_cosp);

  return rpy;
}

void FloatingBaseTrajectoryManager::updateDesired() {
  com_task_->updateDesired(com_pos_des_, com_vel_des_, com_acc_des_);
  base_ori_task_->updateDesired(base_ori_des_, base_ang_vel_des_,
                                base_ang_acc_des_);
  /*// TEST
  double dcm_omega = sqrt(9.81 / robot_->getCoMPosition()[2]);
  dcm_pos_des_ = com_pos_des_ + com_vel_des_ / dcm_omega;
  dcm_vel_des_ = com_vel_des_ + com_acc_des_ / dcm_omega;
  dcm_pos_des_[2] = com_pos_des_[2];
  dcm_vel_des_[2] = com_vel_des_[2];
  dcm_acc_des_.setZero();
  // com_task_->updateDesired(dcm_pos_des_, dcm_vel_des_, dcm_acc_des_);
  // TEST*/
}

void FloatingBaseTrajectoryManager::initializeFloatingBaseTrajectory(
    const double _start_time, const double _end_time,
    const Eigen::VectorXd& _target_com_pos) {
  start_time_ = _start_time;
  duration_ = _end_time - _start_time;
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
  // com_acc_des_ = Eigen::VectorXd::Zero(3);

  base_ori_des_ = ini_base_quat_;
  base_ang_vel_des_ = Eigen::VectorXd::Zero(3);
  base_ang_acc_des_ = Eigen::VectorXd::Zero(3);

  updateDesired();
}
