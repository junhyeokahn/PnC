#include <PnC/TrajectoryManager/FloatingBaseTrajectoryManager.hpp>
#include <PnC/ConvexMPC/ConvexMPC.hpp>
#include <PnC/ConvexMPC/GaitScheduler.hpp>
#include <PnC/A1PnC/A1Definition.hpp>
#include <cmath>
#include <Eigen/Dense>

FloatingBaseTrajectoryManager::FloatingBaseTrajectoryManager(
    Task* _com_task, Task* _base_ori_task, RobotSystem* _robot,
    ConvexMPC* _mpc_planner, GaitScheduler* _gait_scheduler)
    : TrajectoryManagerBase(_robot) {
  myUtils::pretty_constructor(2, "TrajectoryManager: Floating Base");

  gait_scheduler_ = _gait_scheduler;
  mpc_planner_ = _mpc_planner;

  com_task_ = _com_task;
  base_ori_task_ = _base_ori_task;
  base_id_ = base_ori_task_->getLinkID();

  com_pos_des_ = Eigen::VectorXd::Zero(3);
  com_vel_des_ = Eigen::VectorXd::Zero(3);
  com_acc_des_ = Eigen::VectorXd::Zero(3);

  mpc_pos_des_ = Eigen::VectorXd::Zero(3);
  mpc_vel_des_ = Eigen::VectorXd::Zero(3);
  mpc_rpy_des_ = Eigen::VectorXd::Zero(3);// TODO
  mpc_rpydot_des_ = Eigen::VectorXd::Zero(3);// TODO
  foot_contact_states = Eigen::VectorXd::Zero(3);
  foot_pos_body_frame = Eigen::VectorXd::Zero(12);
  foot_friction_coeffs = Eigen::VectorXd::Zero(4);
  foot_friction_coeffs << 0.3, 0.3, 0.3, 0.3;

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

Eigen::Vector3d FloatingBaseTrajectoryManager::toRPY(Eigen::Quaternion<double> quat){
  Eigen::Vector3d rpy;

  double sinr_cosp = 2 * (quat.w() * quat.x() + quat.y() * quat.z());
  double cosr_cosp = 1 - 2 * (quat.x() * quat.x() + quat.y() * quat.y());
  rpy [0] = std::atan2(sinr_cosp, cosr_cosp);

  double sinp = 2 * (quat.w() * quat.y() - quat.z() * quat.x());
  if(std::abs(sinp) >- 1)
    rpy[1] = std::copysign(M_PI / 2, sinp);
  else
    rpy[1] = std::asin(sinp);

  double siny_cosp = 2 * (quat.w() * quat.z() + quat.x() * quat.y());
  double cosy_cosp = 1 - 2 * (quat.y() * quat.y() + quat.z() * quat.z());
  rpy[2] = std::atan2(siny_cosp, cosy_cosp);

  return rpy;

}

void FloatingBaseTrajectoryManager::solveMPC(bool b_fl_contact, bool b_fr_contact,
                                             bool b_rl_contact, bool b_rr_contact,
                                             Eigen::VectorXd com_vel_des,
                                             double target_height,
                                             Eigen::VectorXd& rxn_forces,
                                             double current_time){
  // Set the contact state in gait scheduler from SP values
  if(b_fl_contact) gait_scheduler_->current_contact_state[0] = 1;
  else gait_scheduler_->current_contact_state[0] = 0;
  if(b_fr_contact) gait_scheduler_->current_contact_state[1] = 1;
  else gait_scheduler_->current_contact_state[1] = 0;
  if(b_rl_contact) gait_scheduler_->current_contact_state[2] = 1;
  else gait_scheduler_->current_contact_state[2] = 0;
  if(b_rr_contact) gait_scheduler_->current_contact_state[3] = 1;
  else gait_scheduler_->current_contact_state[3] = 0;
  // Call the gait scheduler to get leg states for next planning step
  gait_scheduler_->step(current_time);
  for(int i=0; i<4; ++i){
    foot_contact_states[i] = gait_scheduler_->leg_state[i];
  }
  // Get Foot position in body frame
  Eigen::Vector3d base_in_world =
       robot_->getBodyNodeIsometry(A1BodyNode::trunk).translation();
  Eigen::Vector3d temp_foot_world =
       robot_->getBodyNodeIsometry(A1BodyNode::FL_foot).translation();
  flfoot_body_frame = temp_foot_world - base_in_world;
  temp_foot_world =
       robot_->getBodyNodeIsometry(A1BodyNode::FR_foot).translation();
  frfoot_body_frame = temp_foot_world - base_in_world;
  temp_foot_world =
       robot_->getBodyNodeIsometry(A1BodyNode::RL_foot).translation();
  rlfoot_body_frame = temp_foot_world - base_in_world;
  temp_foot_world =
       robot_->getBodyNodeIsometry(A1BodyNode::RR_foot).translation();
  rrfoot_body_frame = temp_foot_world - base_in_world;
  foot_pos_body_frame <<
      flfoot_body_frame[0], flfoot_body_frame[1], flfoot_body_frame[2],
      frfoot_body_frame[0], frfoot_body_frame[1], frfoot_body_frame[2],
      rlfoot_body_frame[0], rlfoot_body_frame[1], rlfoot_body_frame[2],
      rrfoot_body_frame[0], rrfoot_body_frame[1], rrfoot_body_frame[2];
  // Set the desired vars for MPC
  // desired position is current position with robot height
  mpc_pos_des_ = robot_->getBodyNodeIsometry(A1BodyNode::trunk).translation();
  mpc_pos_des_[2] = target_height;
  // this is our controlled var
  mpc_vel_des_ = com_vel_des_;
  // get the robot base frame rpy
  Eigen::Quaternion<double> mpc_quat = Eigen::Quaternion<double>(
    robot_->getBodyNodeIsometry(A1BodyNode::trunk).linear());
  mpc_rpy_des_ = toRPY(mpc_quat);
  // our other controlled var
  mpc_rpydot_des_[0] = 0.; mpc_rpydot_des_[1] = 0.;
  mpc_rpydot_des_[2] = base_ang_vel_des_[2];

  rxn_forces = mpc_planner_->ComputeContactForces(
    robot_->getBodyNodeIsometry(A1BodyNode::trunk).translation(), // com pos
    robot_->getBodyNodeSpatialVelocity(A1BodyNode::trunk).tail(3), // com vel
    Eigen::Quaternion<double>(robot_->getBodyNodeIsometry(A1BodyNode::trunk).linear()), //com rpy
    robot_->getBodyNodeCoMSpatialVelocity(A1BodyNode::trunk).head(3), // com rpydot
    foot_contact_states, // foot contact states
    foot_pos_body_frame, // foot pos in body frame
    foot_friction_coeffs, // friction
    mpc_pos_des_, // val set above
    mpc_vel_des_, // val set abovbe (control var)
    mpc_rpy_des_, // val set above
    mpc_rpydot_des_); // val set above (Control var)
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

double FloatingBaseTrajectoryManager::getSwingTime(){
    return gait_scheduler_->swing_duration[0];
}

void FloatingBaseTrajectoryManager::initializeFloatingBaseTrajectory(
    const double _start_time,
    const Eigen::VectorXd& _target_com_pos) {
  start_time_ = _start_time;
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
