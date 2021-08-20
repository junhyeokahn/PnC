#include "pnc/draco_pnc/draco_task/draco_com_task.hpp"

DracoCenterOfMassTask::DracoCenterOfMassTask(RobotSystem *_robot)
    : Task(_robot, 3) {

  util::PrettyConstructor(3, "DracoCenterOfMassTask ");
  sp_ = DracoStateProvider::getStateProvider();
}

void DracoCenterOfMassTask::update_cmd() {

  pos = robot_->get_com_pos();
  pos_err = pos_des - pos;
  vel = sp_->com_vel_est;

  rot_world_local_ =
      robot_->get_link_iso(robot_->get_base_link_name()).linear();

  local_pos_err = rot_world_local_.transpose() * pos_err;

  op_cmd =
      acc_des +
      rot_world_local_ *
          (kp.cwiseProduct(rot_world_local_.transpose() * pos_err) +
           kd.cwiseProduct(rot_world_local_.transpose() * (vel_des - vel)));
}

void DracoCenterOfMassTask::update_jacobian() {
  jacobian = robot_->get_com_lin_jacobian();
  jacobian_dot_q_dot = robot_->get_com_lin_jacobian_dot() * robot_->get_q_dot();
}
