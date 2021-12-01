#include "pnc/whole_body_controllers/basic_tasks.hpp"

#include <assert.h>

#include "configuration.hpp"
#include "utils/util.hpp"

JointTask::JointTask(RobotSystem *_robot) : Task(_robot, _robot->n_a) {

  util::PrettyConstructor(3, "JointTask ");
}

void JointTask::update_cmd(Eigen::Matrix3d rot_world_local) {
  local_pos_des = pos_des;
  local_vel_des = vel_des;
  local_acc_des = acc_des;

  pos = robot_->joint_positions;
  local_pos = pos;

  vel = robot_->joint_velocities;
  local_vel = vel;

  pos_err = pos_des - pos;
  local_pos_err = pos_err;

  vel_err = vel_des - vel;
  local_vel_err = vel_err;

  op_cmd = acc_des + kp.cwiseProduct(pos_err) + kd.cwiseProduct(vel_err);
}

void JointTask::update_jacobian() {
  jacobian.block(0, robot_->n_floating, dim, robot_->n_q_dot) =
      Eigen::MatrixXd::Zero(dim, robot_->n_q_dot);
  jacobian_dot_q_dot = Eigen::VectorXd::Zero(dim);
}

SelectedJointTask::SelectedJointTask(RobotSystem *_robot,
                                     std::vector<std::string> _target_ids)
    : Task(_robot, _target_ids.size(), _target_ids) {

  util::PrettyConstructor(3, "SelectedJointTask ");
}

void SelectedJointTask::update_cmd(Eigen::Matrix3d rot_world_local) {
  local_pos_des = pos_des;
  local_vel_des = vel_des;
  local_acc_des = acc_des;

  for (int i = 0; i < target_ids.size(); ++i) {
    pos[i] = robot_->joint_positions[robot_->get_joint_idx(target_ids[i])];
    vel[i] = robot_->joint_velocities[robot_->get_joint_idx(target_ids[i])];
  }
  local_pos = pos;
  local_vel = vel;

  pos_err = pos_des - pos;
  local_pos_err = pos_err;

  vel_err = vel_des - vel;
  local_vel_err = vel_err;

  op_cmd = acc_des + kp.cwiseProduct(pos_err) + kd.cwiseProduct(vel_err);
}

void SelectedJointTask::update_jacobian() {
  for (int i = 0; i < target_ids.size(); ++i) {
    jacobian(i, robot_->get_q_dot_idx(target_ids[i])) = 1.;
  }
  jacobian_dot_q_dot = Eigen::VectorXd::Zero(dim);
}

LinkPosTask::LinkPosTask(RobotSystem *_robot,
                         std::vector<std::string> _target_ids)
    : Task(_robot, 3 * _target_ids.size(), _target_ids) {
  assert(dim == 3 * _target_ids.size());

  util::PrettyConstructor(3, "LinkPosTask ");
}

void LinkPosTask::update_cmd(Eigen::Matrix3d rot_world_local) {

  Eigen::MatrixXd aug_rot_world_local = Eigen::MatrixXd::Zero(dim, dim);
  for (int i = 0; i < target_ids.size(); ++i) {
    pos.segment(3 * i, 3) = robot_->get_link_iso(target_ids[i]).translation();
    vel.segment(3 * i, 3) = robot_->get_link_vel(target_ids[i]).tail(3);
    aug_rot_world_local.block(3 * i, 3 * i, 3, 3) = rot_world_local;
  }
  local_pos_des = aug_rot_world_local.transpose() * pos_des;
  local_vel_des = aug_rot_world_local.transpose() * vel_des;
  local_acc_des = aug_rot_world_local.transpose() * acc_des;
  local_pos = aug_rot_world_local.transpose() * pos;
  local_vel = aug_rot_world_local.transpose() * vel;

  pos_err = pos_des - pos;
  local_pos_err = aug_rot_world_local.transpose() * pos_err;
  vel_err = vel_des - vel;
  local_vel_err = aug_rot_world_local.transpose() * (vel_err);

  op_cmd = acc_des + aug_rot_world_local * (kp.cwiseProduct(local_pos_err) +
                                            kd.cwiseProduct(local_vel_err));
}

void LinkPosTask::update_jacobian() {
  for (int i = 0; i < target_ids.size(); ++i) {
    jacobian.block(3 * i, 0, 3, robot_->n_q_dot) =
        robot_->get_link_jacobian(target_ids[i])
            .block(3, 0, 3, robot_->n_q_dot);
    jacobian_dot_q_dot.segment(3 * i, 3) =
        robot_->get_link_jacobian_dot_times_qdot(target_ids[i]).tail(3);
  }
}

LinkOriTask::LinkOriTask(RobotSystem *_robot,
                         std::vector<std::string> _target_ids)
    : Task(_robot, 3 * _target_ids.size(), _target_ids) {
  assert(dim == 3 * _target_ids.size());
  pos_des = Eigen::VectorXd::Zero(4 * _target_ids.size());
  pos = Eigen::VectorXd::Zero(4 * _target_ids.size());

  local_pos_des = Eigen::VectorXd::Zero(4 * _target_ids.size());
  local_pos = Eigen::VectorXd::Zero(4 * _target_ids.size());

  util::PrettyConstructor(3, "LinkOriTask ");
}

void LinkOriTask::update_cmd(Eigen::Matrix3d rot_world_local) {

  Eigen::MatrixXd aug_rot_world_local = Eigen::MatrixXd::Zero(dim, dim);
  for (int i = 0; i < target_ids.size(); ++i) {
    Eigen::Quaternion<double> quat_des(pos_des[4 * i + 0], pos_des[4 * i + 1],
                                       pos_des[4 * i + 2], pos_des[4 * i + 3]);
    Eigen::Quaternion<double> local_quat_des = Eigen::Quaternion<double>(
        rot_world_local.transpose() * quat_des.toRotationMatrix());
    local_pos_des[4 * i + 0] = local_quat_des.w();
    local_pos_des[4 * i + 1] = local_quat_des.x();
    local_pos_des[4 * i + 2] = local_quat_des.y();
    local_pos_des[4 * i + 3] = local_quat_des.z();

    Eigen::Quaternion<double> quat(
        robot_->get_link_iso(target_ids[i]).linear());
    util::AvoidQuatJump(quat_des, quat);
    Eigen::Quaternion<double> local_quat = Eigen::Quaternion<double>(
        rot_world_local.transpose() * quat.toRotationMatrix());
    local_pos[4 * i + 0] = local_quat.w();
    local_pos[4 * i + 1] = local_quat.x();
    local_pos[4 * i + 2] = local_quat.y();
    local_pos[4 * i + 3] = local_quat.z();

    Eigen::Quaternion<double> quat_err = quat_des * quat.inverse();
    Eigen::Vector3d ori_err = util::QuatToExp(quat_err);
    pos.segment(4 * i, 4) =
        Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z());
    pos_err.segment(3 * i, 3) = ori_err;
    vel.segment(3 * i, 3) = robot_->get_link_vel(target_ids[i]).head(3);
    aug_rot_world_local.block(3 * i, 3 * i, 3, 3) = rot_world_local;
  }

  local_vel_des = aug_rot_world_local.transpose() * vel_des;
  local_vel = aug_rot_world_local.transpose() * vel;

  local_acc_des = aug_rot_world_local.transpose() * acc_des;

  local_pos_err = aug_rot_world_local.transpose() * pos_err;

  vel_err = vel_des - vel;
  local_vel_err = local_vel_des - local_vel;

  op_cmd = acc_des + aug_rot_world_local * (kp.cwiseProduct(local_pos_err) +
                                            kd.cwiseProduct(local_vel_err));
}

void LinkOriTask::update_jacobian() {
  for (int i = 0; i < target_ids.size(); ++i) {
    jacobian.block(3 * i, 0, 3, robot_->n_q_dot) =
        robot_->get_link_jacobian(target_ids[i])
            .block(0, 0, 3, robot_->n_q_dot);
    jacobian_dot_q_dot.segment(3 * i, 3) =
        robot_->get_link_jacobian_dot_times_qdot(target_ids[i]).head(3);
  }
}

CenterOfMassTask::CenterOfMassTask(RobotSystem *_robot) : Task(_robot, 3) {

  util::PrettyConstructor(3, "CenterOfMassTask ");
}

void CenterOfMassTask::update_cmd(Eigen::Matrix3d rot_world_local) {
  local_pos_des = rot_world_local.transpose() * pos_des;
  local_vel_des = rot_world_local.transpose() * vel_des;
  local_acc_des = rot_world_local.transpose() * acc_des;

  pos = robot_->get_com_pos();
  local_pos = rot_world_local.transpose() * pos;

  pos_err = pos_des - pos;
  local_pos_err = rot_world_local.transpose() * pos_err;

  vel = robot_->get_com_lin_vel();
  local_vel = rot_world_local.transpose() * vel;

  vel_err = vel_des - vel;
  local_vel_err = rot_world_local.transpose() * vel_err;

  op_cmd =
      acc_des + rot_world_local *
                    (kp.cwiseProduct(rot_world_local.transpose() * pos_err) +
                     kd.cwiseProduct(rot_world_local.transpose() * vel_err));
}

void CenterOfMassTask::update_jacobian() {
  jacobian = robot_->get_com_lin_jacobian();
  jacobian_dot_q_dot = robot_->get_com_lin_jacobian_dot() * robot_->get_q_dot();
}
