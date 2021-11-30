#include "pnc/draco_pnc/draco_task/draco_angular_momentum_task.hpp"

DracoAngularMomentumTask::DracoAngularMomentumTask(RobotSystem *_robot)
    : Task(_robot, 3) {

  util::PrettyConstructor(3, "DracoAngularMomentumTask");
  sp_ = DracoStateProvider::getStateProvider();
}

void DracoAngularMomentumTask::update_cmd(Eigen::Matrix3d rot_world_local) {

  vel = sp_->cam_est;

  local_vel_err = rot_world_local.transpose() * (vel_des - vel);

  op_cmd = acc_des + rot_world_local * kd.cwiseProduct(local_vel_err);
}

void DracoAngularMomentumTask::update_jacobian() {
  jacobian = robot_->Ag.block(0, 0, dim, robot_->n_q_dot);
  jacobian_dot_q_dot = Eigen::VectorXd::Zero(dim);
}
