#include "pnc/whole_body_controllers/managers/end_effector_trajectory_manager.hpp"

EndEffectorTrajectoryManager::EndEffectorTrajectoryManager(
    Task *_pos_task, Task *_ori_task, RobotSystem *_robot) {

  util::PrettyConstructor(2, "TrajectoryManager: End-effector");

  robot_ = _robot;

  pos_task_ = _pos_task;
  ori_task_ = _ori_task;
  link_idx_ = pos_task_->target_ids[0];

  amp_.setZero();
  freq_.setZero();
  Eigen::Vector3d exp_error_;

  b_swaying_ = false;
}

EndEffectorTrajectoryManager::~EndEffectorTrajectoryManager() {}

void EndEffectorTrajectoryManager::UseCurrent() {
  Eigen::VectorXd pos_des = robot_->get_link_iso(link_idx_).translation();
  Eigen::VectorXd vel_des = robot_->get_link_vel(link_idx_).tail(3);
  Eigen::VectorXd acc_des = Eigen::VectorXd::Zero(3);

  Eigen::Quaternion<double> ori_des_quat =
      Eigen::Quaternion<double>(robot_->get_link_iso(link_idx_).linear());
  Eigen::VectorXd ori_des(4);
  ori_des << ori_des_quat.w(), ori_des_quat.x(), ori_des_quat.y(),
      ori_des_quat.z();
  Eigen::Vector3d ang_vel_des = robot_->get_link_vel(link_idx_).head(3);

  pos_task_->update_desired(pos_des, vel_des, acc_des);
  ori_task_->update_desired(ori_des, ang_vel_des, acc_des);
}

void EndEffectorTrajectoryManager::InitializeInterpolationTrajectory(
    const double &_start_time, const double &_duration,
    const Eigen::Vector3d &_target_pos,
    const Eigen::Quaternion<double> &_target_quat) {
  start_time_ = _start_time;
  duration_ = _duration;

  ini_pos_ = robot_->get_link_iso(link_idx_).translation();
  ini_quat_ =
      Eigen::Quaternion<double>(robot_->get_link_iso(link_idx_).linear());

  target_pos_ = _target_pos;
  target_quat_ = _target_quat;

  Eigen::Quaternion<double> quat_err = target_quat_ * ini_quat_.inverse();
  exp_error_ = util::QuatToExp(quat_err);
}

void EndEffectorTrajectoryManager::InitializeSwayingTrajectory(
    const double &_start_time, const Eigen::Vector3d &_amp,
    const Eigen::Vector3d &_freq) {
  b_swaying_ = true;
  start_time_ = _start_time;
  amp_ = _amp;
  freq_ = _freq;

  ini_pos_ = robot_->get_link_iso(link_idx_).translation();
  ini_quat_ =
      Eigen::Quaternion<double>(robot_->get_link_iso(link_idx_).linear());
  target_quat_ = ini_quat_;
}

void EndEffectorTrajectoryManager::UpdateDesired(const double &_curr_time) {
  Eigen::VectorXd pos_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd vel_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd acc_des = Eigen::VectorXd::Zero(3);

  if (b_swaying_) {
    util::SinusoidTrajectory(start_time_, ini_pos_, amp_, freq_, _curr_time,
                             pos_des, vel_des, acc_des);
  } else {
    for (int i = 0; i < 3; ++i) {
      pos_des[i] = util::SmoothPos(ini_pos_[i], target_pos_[i], duration_,
                                   _curr_time - start_time_);
      vel_des[i] = util::SmoothVel(ini_pos_[i], target_pos_[i], duration_,
                                   _curr_time - start_time_);
      acc_des[i] = util::SmoothAcc(ini_pos_[i], target_pos_[i], duration_,
                                   _curr_time - start_time_);
    }
  }

  pos_task_->update_desired(pos_des, vel_des, acc_des);

  Eigen::VectorXd ori_des = Eigen::VectorXd::Zero(4);
  Eigen::Quaternion<double> ori_des_quat;
  Eigen::VectorXd ang_vel_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd ang_acc_des = Eigen::VectorXd::Zero(3);

  double scaled_t = util::SmoothPos(0, 1, duration_, _curr_time - start_time_);
  double scaled_tdot =
      util::SmoothVel(0, 1, duration_, _curr_time - start_time_);
  double scaled_tddot =
      util::SmoothAcc(0, 1, duration_, _curr_time - start_time_);

  Eigen::Vector3d exp_inc = exp_error_ * scaled_t;
  Eigen::Quaternion<double> quat_inc = util::ExpToQuat(exp_inc);
  // TODO (Check this again)
  // ori_des_quat = quat_inc * ini_quat_;
  ori_des_quat = ini_quat_ * quat_inc;
  ori_des << ori_des_quat.w(), ori_des_quat.x(), ori_des_quat.y(),
      ori_des_quat.z();
  ang_vel_des = exp_error_ * scaled_tdot;
  ang_acc_des = exp_error_ * scaled_tddot;

  ori_task_->update_desired(ori_des, ang_vel_des, ang_acc_des);
}
