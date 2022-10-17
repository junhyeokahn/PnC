#include "pnc/whole_body_controllers/managers/floating_base_trajectory_manager.hpp"

FloatingBaseTrajectoryManager::FloatingBaseTrajectoryManager(
    Task *_com_task, Task *_base_ori_task, RobotSystem *_robot,
    bool _b_use_base_height) {
  util::PrettyConstructor(2, "TrajectoryManager: Floating Base");

  robot_ = _robot;

  com_task_ = _com_task;
  base_ori_task_ = _base_ori_task;

  base_id_ = base_ori_task_->target_ids[0];

  local_amp_.setZero();
  local_freq_.setZero();
  exp_error_.setZero();

  rot_world_local_.setIdentity();

  b_swaying_ = false;
  b_use_base_height_ = _b_use_base_height;
  ini_base_quat_.setIdentity();
}

void FloatingBaseTrajectoryManager::InitializeInterpolationTrajectory(
    const double _start_time, const double _duration,
    const Eigen::Vector3d &_target_com_pos, const Eigen::Vector3d &des_com_pos,
    const Eigen::Quaternion<double> &_target_base_quat,
    const Eigen::Quaternion<double> &des_ini_base_quat) {

  b_swaying_ = false;

  start_time_ = _start_time;
  duration_ = _duration;

  // ini_com_pos_ = robot_->get_com_pos();
  // if (b_use_base_height_) {
  // ini_com_pos_[2] = robot_->get_link_iso(base_id_).translation()[2];
  //}
  ini_com_pos_ = des_com_pos;
  // ini_base_quat_ =
  // Eigen::Quaternion<double>(robot_->get_link_iso(base_id_).linear());
  ini_base_quat_ = des_ini_base_quat;

  target_com_pos_ = _target_com_pos;
  target_base_quat_ = _target_base_quat;

  Eigen::Quaternion<double> quat_err =
      target_base_quat_ * ini_base_quat_.inverse();
  exp_error_ = util::QuatToExp(quat_err);
}

void FloatingBaseTrajectoryManager::InitializeInterpolationTrajectory(
    const double _start_time, const double _duration,
    const Eigen::Vector3d &_target_com_pos,
    const Eigen::Quaternion<double> &_target_base_quat) {

  b_swaying_ = false;

  start_time_ = _start_time;
  duration_ = _duration;

  ini_com_pos_ = robot_->get_com_pos();
  if (b_use_base_height_) {
    ini_com_pos_[2] = robot_->get_link_iso(base_id_).translation()[2];
  }
  ini_base_quat_ =
      Eigen::Quaternion<double>(robot_->get_link_iso(base_id_).linear());

  target_com_pos_ = _target_com_pos;
  target_base_quat_ = _target_base_quat;

  Eigen::Quaternion<double> quat_err =
      target_base_quat_ * ini_base_quat_.inverse();
  exp_error_ = util::QuatToExp(quat_err);
}

void FloatingBaseTrajectoryManager::InitializeSwayingTrajectory(
    double _start_time, const Eigen::Vector3d &_amp,
    const Eigen::Vector3d &_freq, const Eigen::Matrix3d &_rot_world_local,
    const Eigen::Vector3d &des_com_pos,
    const Eigen::Quaternion<double> &des_quat) {

  b_swaying_ = true;

  start_time_ = _start_time;
  local_amp_ = _amp;
  local_freq_ = _freq;
  rot_world_local_ = _rot_world_local;

  // ini_com_pos_ = robot_->get_com_pos();
  // if (b_use_base_height_) {
  // ini_com_pos_[2] = robot_->get_link_iso(base_id_).translation()[2];
  //}
  ini_com_pos_ = des_com_pos;

  // ini_base_quat_ =
  // Eigen::Quaternion<double>(robot_->get_link_iso(base_id_).linear());
  target_base_quat_ = des_quat;
}

void FloatingBaseTrajectoryManager::UpdateDesired(const double curr_time) {

  Eigen::VectorXd com_pos_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd com_vel_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd com_acc_des = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd base_ori_des = Eigen::VectorXd::Zero(4);
  Eigen::Quaternion<double> base_ori_des_quat;
  Eigen::VectorXd base_ang_vel_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd base_ang_acc_des = Eigen::VectorXd::Zero(3);

  if (b_swaying_) {
    Eigen::VectorXd local_pos_des, local_vel_des, local_acc_des;
    util::SinusoidTrajectory(start_time_, Eigen::VectorXd::Zero(3), local_amp_,
                             local_freq_, curr_time, local_pos_des,
                             local_vel_des, local_acc_des);
    com_pos_des = ini_com_pos_ + rot_world_local_ * local_pos_des;
    // com_pos_des[2] = ini_com_pos_[2]; // maintain constant height
    com_vel_des = rot_world_local_ * local_vel_des;
    com_acc_des = rot_world_local_ * local_acc_des;
    base_ori_des << target_base_quat_.w(), target_base_quat_.x(),
        target_base_quat_.y(), target_base_quat_.z();
    base_ang_vel_des = Eigen::VectorXd::Zero(3);
    base_ang_acc_des = Eigen::VectorXd::Zero(3);
  } else {
    for (int i = 0; i < 3; ++i) {
      com_pos_des[i] = util::SmoothPos(ini_com_pos_[i], target_com_pos_[i],
                                       duration_, curr_time - start_time_);
      com_vel_des[i] = util::SmoothVel(ini_com_pos_[i], target_com_pos_[i],
                                       duration_, curr_time - start_time_);
      com_acc_des[i] = util::SmoothAcc(ini_com_pos_[i], target_com_pos_[i],
                                       duration_, curr_time - start_time_);
    }
    double scaled_t = util::SmoothPos(0, 1, duration_, curr_time - start_time_);
    double scaled_tdot =
        util::SmoothVel(0, 1, duration_, curr_time - start_time_);
    double scaled_tddot =
        util::SmoothAcc(0, 1, duration_, curr_time - start_time_);

    Eigen::Vector3d exp_inc = exp_error_ * scaled_t;
    Eigen::Quaternion<double> quat_inc = util::ExpToQuat(exp_inc);
    base_ori_des_quat = quat_inc * ini_base_quat_;
    base_ori_des << base_ori_des_quat.w(), base_ori_des_quat.x(),
        base_ori_des_quat.y(), base_ori_des_quat.z();
    base_ang_vel_des = exp_error_ * scaled_tdot;
    base_ang_acc_des = exp_error_ * scaled_tddot;
  }

  com_task_->update_desired(com_pos_des, com_vel_des, com_acc_des);
  base_ori_task_->update_desired(base_ori_des, base_ang_vel_des,
                                 base_ang_acc_des);
}
