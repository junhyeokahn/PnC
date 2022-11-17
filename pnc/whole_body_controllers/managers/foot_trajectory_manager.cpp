#include "pnc/whole_body_controllers/managers/foot_trajectory_manager.hpp"

FootTrajectoryManager::FootTrajectoryManager(Task *_foot_pos_task,
                                             Task *_foot_ori_task,
                                             RobotSystem *_robot) {
  util::PrettyConstructor(2, "TrajectoryManager: Foot");

  robot_ = _robot;
  foot_pos_task_ = _foot_pos_task;
  foot_ori_task_ = _foot_ori_task;

  link_idx_ = foot_pos_task_->target_ids[0];

  ini_pos_.setZero();
  target_pos_.setZero();
  ini_quat_.setIdentity();
  target_quat_.setIdentity();
  exp_error_.setZero();

  swing_height = 0.04;
}

FootTrajectoryManager::~FootTrajectoryManager() {}

void FootTrajectoryManager::UpdateZeroAccCmd() {
  Eigen::VectorXd foot_pos_des = robot_->get_link_iso(link_idx_).translation();
  Eigen::VectorXd foot_vel_des = robot_->get_link_vel(link_idx_).tail(3);
  Eigen::VectorXd foot_acc_des = Eigen::VectorXd::Zero(3);

  Eigen::Quaternion<double> foot_quat_des =
      Eigen::Quaternion<double>(robot_->get_link_iso(link_idx_).linear());
  Eigen::VectorXd foot_ori_des(4);
  foot_ori_des << foot_quat_des.w(), foot_quat_des.x(), foot_quat_des.y(),
      foot_quat_des.z();
  Eigen::VectorXd foot_ang_vel_des = robot_->get_link_vel(link_idx_).head(3);

  foot_pos_task_->update_desired(foot_pos_des, foot_vel_des, foot_acc_des);
  foot_ori_task_->update_desired(foot_ori_des, foot_ang_vel_des, foot_acc_des);
}

void FootTrajectoryManager::useNominalPoseCmd(
    const Eigen::Isometry3d &nominal_foot_iso) {
  Eigen::VectorXd foot_pos_des = nominal_foot_iso.translation();
  Eigen::VectorXd foot_vel_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd foot_acc_des = Eigen::VectorXd::Zero(3);

  Eigen::Quaternion<double> foot_quat_des =
      Eigen::Quaternion<double>(nominal_foot_iso.linear());
  Eigen::VectorXd foot_ori_des(4);
  foot_ori_des << foot_quat_des.w(), foot_quat_des.x(), foot_quat_des.y(),
      foot_quat_des.z();
  Eigen::VectorXd foot_ang_vel_des = Eigen::VectorXd::Zero(3);

  foot_pos_task_->update_desired(foot_pos_des, foot_vel_des, foot_acc_des);
  foot_ori_task_->update_desired(foot_ori_des, foot_ang_vel_des, foot_acc_des);
}

Eigen::Vector3d FootTrajectoryManager::GetDesiredPos() const {
  return foot_pos_task_->pos_des;
}

Eigen::Quaternion<double> FootTrajectoryManager::GetDesiredOri() const {
  Eigen::Quaternion<double> quat = Eigen::Quaternion<double>::Identity();
  Eigen::VectorXd foot_ori = foot_ori_task_->pos_des;
  quat.w() = foot_ori[0];
  quat.vec() = foot_ori.tail<3>();
  return quat;
}

void FootTrajectoryManager::InitializeSwingTrajectory(
    const double _start_time, const double _swing_duration,
    const Footstep &_landing_foot) {

  this->InitializeSwingTrajectory(_start_time, _swing_duration,
                                  robot_->get_link_iso(link_idx_),
                                  _landing_foot);
}

void FootTrajectoryManager::InitializeSwingTrajectory(
    const double _start_time, const double _swing_duration,
    const Eigen::Isometry3d &_start_foot_iso, const Footstep &_landing_foot) {

  // Copy and initialize variables
  start_time_ = _start_time;
  duration_ = _swing_duration;
  swing_land_foot_ = _landing_foot;

  // initialize swing foot starting pose
  Eigen::Vector3d start_foot_pos = _start_foot_iso.translation();
  Eigen::Quaterniond start_foot_ori(_start_foot_iso.linear());

  swing_init_foot_.setPosOriSide(start_foot_pos, start_foot_ori,
                                 swing_land_foot_.robot_side);

  // Compute where the foot will be in the middle of the trajectory
  swing_midfoot_.computeMidfeet(swing_init_foot_, swing_land_foot_,
                                swing_midfoot_);

  // Compute midfeet boundary conditions
  // Linear velocity at the middle of the swing is the total swing travel over
  // swing time
  Eigen::Vector3d mid_swing_local_foot_pos(0, 0, swing_height);
  Eigen::Vector3d mid_swing_position =
          swing_midfoot_.position + swing_midfoot_.R_ori * mid_swing_local_foot_pos;
  Eigen::Vector3d mid_swing_velocity =
          (swing_land_foot_.position - swing_init_foot_.position) / duration_;

  // Construct Position trajectories
  pos_traj_init_to_mid_.initialize(swing_init_foot_.position,
                                   Eigen::Vector3d::Zero(3), mid_swing_position,
                                   mid_swing_velocity, 0.5 * duration_);
  pos_traj_mid_to_end_.initialize(mid_swing_position, mid_swing_velocity,
                                  swing_land_foot_.position,
                                  Eigen::Vector3d::Zero(3), 0.5 * duration_);

  // Construct Quaternion trajectory
  Eigen::Vector3d ang_vel_start;
  ang_vel_start.setZero();
  Eigen::Vector3d ang_vel_end;
  ang_vel_end.setZero();

  quat_hermite_curve_.initialize(swing_init_foot_.orientation, ang_vel_start,
                                 swing_land_foot_.orientation, ang_vel_end,
                                 duration_);
}

void FootTrajectoryManager::UpdateDesired(const double current_time) {

  Eigen::VectorXd pos_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd vel_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd acc_des = Eigen::VectorXd::Zero(3);

  Eigen::Quaternion<double> quat_des;
  Eigen::VectorXd ori_des = Eigen::VectorXd::Zero(4);
  Eigen::Vector3d ang_vel_des;
  Eigen::Vector3d ang_acc_des;

  // double s = (current_time - start_time_) / duration_;

  // if (s <= 0.5) {
  // s = 2.0 * s;
  // pos_des = pos_traj_init_to_mid_.Evaluate(s);
  // vel_des =
  // pos_traj_init_to_mid_.EvaluateFirstDerivative(s) / (duration_ * 0.5);
  // acc_des =
  // pos_traj_init_to_mid_.EvaluateSecondDerivative(s) / (duration_ * 0.5);
  //} else {
  // s = 2.0 * (s - 0.5);
  // pos_des = pos_traj_mid_to_end_.Evaluate(s);
  // vel_des =
  // pos_traj_mid_to_end_.EvaluateFirstDerivative(s) / (duration_ * 0.5);
  // acc_des =
  // pos_traj_mid_to_end_.EvaluateSecondDerivative(s) / (duration_ * 0.5);
  //}

  // s = (current_time - start_time_) / duration_;

  // quat_hermite_curve_.Evaluate(s, quat_des);
  // ori_des << quat_des.w(), quat_des.x(), quat_des.y(), quat_des.z();
  // quat_hermite_curve_.GetAngularVelocity(s, ang_vel_des);
  // quat_hermite_curve_.GetAngularAcceleration(s, ang_acc_des);
  // ang_vel_des /= duration_;
  // ang_acc_des /= duration_;
  double s = current_time - start_time_;
  if (s < 0.5 * duration_) {
    pos_des = pos_traj_init_to_mid_.evaluate(s);
    vel_des = pos_traj_init_to_mid_.evaluateFirstDerivative(s);
    acc_des = pos_traj_init_to_mid_.evaluateSecondDerivative(s);
  } else {
    pos_des = pos_traj_mid_to_end_.evaluate(s - 0.5 * duration_);
    vel_des = pos_traj_mid_to_end_.evaluateFirstDerivative(s - 0.5 * duration_);
    acc_des =
        pos_traj_mid_to_end_.evaluateSecondDerivative(s - 0.5 * duration_);
  }

  quat_hermite_curve_.evaluate(s, quat_des);
  ori_des << quat_des.w(), quat_des.x(), quat_des.y(), quat_des.z();
  quat_hermite_curve_.getAngularVelocity(s, ang_vel_des);
  quat_hermite_curve_.getAngularAcceleration(s, ang_acc_des);

  foot_pos_task_->update_desired(pos_des, vel_des, acc_des);
  foot_ori_task_->update_desired(ori_des, ang_vel_des, ang_acc_des);
}

// foot trajectory interpolation
void FootTrajectoryManager::InitializeInterpolationTrajectory(
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

void FootTrajectoryManager::InitializeInterpolationTrajectory(
    const double &_start_time, const double &_duration,
    const Eigen::Vector3d &_target_pos, const Eigen::Vector3d &_init_pos,
    const Eigen::Quaternion<double> &_target_quat,
    const Eigen::Quaternion<double> &_init_quat) {
  start_time_ = _start_time;
  duration_ = _duration;

  ini_pos_ = _init_pos;
  ini_quat_ = _init_quat;

  target_pos_ = _target_pos;
  target_quat_ = _target_quat;

  Eigen::Quaternion<double> quat_err = target_quat_ * ini_quat_.inverse();
  exp_error_ = util::QuatToExp(quat_err);
}
void FootTrajectoryManager::UpdateInterpolationDesired(
    const double &_curr_time) {
  Eigen::VectorXd pos_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd vel_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd acc_des = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd ori_des = Eigen::VectorXd::Zero(4);
  Eigen::Quaternion<double> ori_des_quat;
  Eigen::VectorXd ang_vel_des = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd ang_acc_des = Eigen::VectorXd::Zero(3);

  for (int i = 0; i < 3; ++i) {
    pos_des[i] = util::SmoothPos(ini_pos_[i], target_pos_[i], duration_,
                                 _curr_time - start_time_);
    vel_des[i] = util::SmoothVel(ini_pos_[i], target_pos_[i], duration_,
                                 _curr_time - start_time_);
    acc_des[i] = util::SmoothAcc(ini_pos_[i], target_pos_[i], duration_,
                                 _curr_time - start_time_);
  }
  double scaled_t = util::SmoothPos(0, 1, duration_, _curr_time - start_time_);
  double scaled_tdot =
      util::SmoothVel(0, 1, duration_, _curr_time - start_time_);
  double scaled_tddot =
      util::SmoothAcc(0, 1, duration_, _curr_time - start_time_);
  Eigen::Vector3d exp_inc = exp_error_ * scaled_t;
  Eigen::Quaternion<double> quat_inc = util::ExpToQuat(exp_inc);
  ori_des_quat = quat_inc * ini_quat_;
  ori_des << ori_des_quat.w(), ori_des_quat.x(), ori_des_quat.y(),
      ori_des_quat.z();
  ang_vel_des = exp_error_ * scaled_tdot;
  ang_acc_des = exp_error_ * scaled_tddot;

  foot_pos_task_->update_desired(pos_des, vel_des, acc_des);
  foot_ori_task_->update_desired(ori_des, ang_vel_des, ang_acc_des);
}
