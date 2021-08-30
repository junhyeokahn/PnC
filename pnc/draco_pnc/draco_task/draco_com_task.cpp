#include "pnc/draco_pnc/draco_task/draco_com_task.hpp"

DracoCenterOfMassTask::DracoCenterOfMassTask(RobotSystem *_robot,
                                             int _feedback_source)
    : Task(_robot, 3) {

  util::PrettyConstructor(3, "DracoCenterOfMassTask");
  sp_ = DracoStateProvider::getStateProvider();
  feedback_source_ = _feedback_source;
  icp_des.setZero();
  icp_dot_des.setZero();
}

void DracoCenterOfMassTask::update_cmd() {

  if (feedback_source_ == feedback_source::kCom) {
    pos = robot_->get_com_pos();
    pos_err = pos_des - pos;
    vel = sp_->com_vel_est;

    rot_world_local_ =
        robot_->get_link_iso(robot_->get_base_link_name()).linear();

    local_pos_err = rot_world_local_.transpose() * pos_err;
    local_pos_err[2] = pos_des[2] - pos[2]; // assume z axis is upright
    local_vel_err = rot_world_local_.transpose() * (vel_des - vel);

    op_cmd = acc_des + rot_world_local_ * (kp.cwiseProduct(local_pos_err) +
                                           kd.cwiseProduct(local_vel_err));
  } else if (feedback_source_ == feedback_source::kIcp) {

    double z_des(pos_des[2]);
    double z_dot_des(vel_des[2]);
    double omega(sqrt(9.81 / z_des));

    icp_des = pos_des.head(2) + vel_des.head(2) / omega;
    icp_dot_des = vel_des.head(2) + acc_des.head(2) / omega;

    Eigen::Vector3d com_pos = robot_->get_com_pos();
    Eigen::Vector3d com_vel = sp_->com_vel_est;
    Eigen::Vector2d icp = sp_->dcm.head(2);

    pos = com_pos;
    vel = com_vel;
    pos_err = pos_des - pos;

    rot_world_local_ =
        robot_->get_link_iso(robot_->get_base_link_name()).linear();

    local_pos_err = rot_world_local_.transpose() * pos_err;
    local_pos_err[2] = pos_des[2] - pos[2]; // assume z axis is upright
    local_vel_err = rot_world_local_.transpose() * (vel_des - vel);

    // TODO : add integral gain
    Eigen::Vector2d cmp_des =
        icp - icp_dot_des / omega - kp.head(2).cwiseProduct(icp_des - icp);

    op_cmd.head(2) = (9.81 / z_des) * (com_pos.head(2) - cmp_des);
    op_cmd[2] = kp[2] * (z_des - com_pos[2]) + kd[2] * (z_dot_des - com_vel[2]);

  } else {
    assert(false);
  }
}

void DracoCenterOfMassTask::update_jacobian() {
  jacobian = robot_->get_com_lin_jacobian();
  jacobian_dot_q_dot = robot_->get_com_lin_jacobian_dot() * robot_->get_q_dot();
}
