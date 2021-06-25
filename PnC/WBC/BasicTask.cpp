#include <assert.h>

#include <Configuration.hpp>
#include <PnC/WBC/BasicTask.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

BasicTask::BasicTask(RobotSystem *_robot, Type _task_type, const int _dim,
                     std::vector<std::string> _target_ids)
    : Task(_robot, _dim, _target_ids) {
  task_type_ = _task_type;
  std::string name;
  for (int i = 0; i < dim; ++i) {
    kp[i] = 0.;
    kd[i] = 0.;
  }

  switch (task_type_) {
  case Type::JOINT:
    assert(dim == robot_->n_a);
    name = "Joint";
    break;
  case Type::SELECTED_JOINT:
    name = "Selected Joint";
    break;
  case Type::LINK_XYZ:
    assert(dim == 3);
    assert(target_ids.size() == 1);
    name = "LinkXYZ";
    break;
  case Type::LINK_ORI:
    assert(dim == 3);
    assert(target_ids.size() == 1);
    name = "LinkORI";
    break;
  case Type::COM:
    assert(dim == 3);
    name = "CoM";
    break;
  case Type::ISOLATED_LINK_XYZ:
    assert(dim == 3);
    assert(target_ids.size() == 1);
    name = "Isolated LinkXYZ";
    break;
  case Type::ISOLATED_LINK_ORI:
    assert(dim == 3);
    assert(target_ids.size() == 1);
    name = "Isolated LinkOri";
    break;
  case Type::ISOLATED_COM:
    assert(dim == 3);
    name = "Isolated CoM";
    break;
  default: { assert(false); }
  }
  myUtils::pretty_constructor(3, "Basic Task " + name);
}

void BasicTask::update_cmd() {
  Eigen::VectorXd pos = Eigen::VectorXd::Zero(dim);
  Eigen::VectorXd vel = Eigen::VectorXd::Zero(dim);
  switch (task_type_) {
  case Type::JOINT: {
    pos = robot_->joint_positions;
    vel = robot_->joint_velocities;
    pos_err = pos_des_ - pos;
    break;
  }
  case Type::SELECTED_JOINT: {
    for (int i = 0; i < target_ids.size(); ++i) {
      pos[i] = robot_->joint_positions[robot_->get_joint_idx(target_ids[i])];
      vel[i] = robot_->joint_velocities[robot_->get_joint_idx(target_ids[i])];
    }
    pos_err = pos_des_ - pos;
    break;
  }
  case Type::ISOLATED_LINK_XYZ:
  case Type::LINK_XYZ: {
    pos = robot_->get_link_iso(target_ids[0]).translation();
    vel = robot_->get_link_vel(target_ids[0]).tail(3);
    pos_err = pos_des_ - pos;
    break;
  }
  case Type::ISOLATED_LINK_ORI:
  case Type::LINK_ORI: {
    Eigen::Quaternion<double> quat_des(pos_des_[0], pos_des_[1], pos_des_[2],
                                       pos_des_[3]);
    Eigen::Quaternion<double> quat(
        robot_->get_link_iso(target_ids[0]).linear());
    myUtils::avoid_quat_jump(quat_des, quat);
    Eigen::Quaternion<double> quat_err = quat_des * quat.inverse();
    Eigen::Vector3d ori_err = myUtils::quat_to_exp(quat_err);
    pos_err = ori_err;
    vel = robot_->get_link_vel(target_ids[0]).head(3);
    break;
  }
  case Type::ISOLATED_COM:
  case Type::COM: {
    pos = robot_->get_com_pos();
    pos_err = pos_des_ - pos;
    vel = robot_->get_com_lin_vel();
    break;
  }
  default: { assert(false); }
  }
  for (int i = 0; i < dim; ++i) {
    op_cmd[i] =
        acc_des_[i] + kp[i] * pos_err[i] + kd[i] * (vel_des_[i] - vel[i]);
  }
}

void BasicTask::update_jacobian() {
  switch (task_type_) {
  case Type::JOINT: {
    jacobian.block(0, robot_->n_floating, dim, robot_->n_q_dot) =
        Eigen::MatrixXd::Zero(dim, robot_->n_q_dot);
    jacobian_dot_q_dot = Eigen::VectorXd::Zero(dim);
    break;
  }
  case Type::SELECTED_JOINT: {
    for (int i = 0; i < target_ids.size(); ++i) {
      jacobian(i, robot_->get_q_dot_idx(target_ids[i])) = 1.;
    }
    jacobian_dot_q_dot = Eigen::VectorXd::Zero(dim);
    break;
  }
  case Type::ISOLATED_LINK_XYZ: {
    jacobian = robot_->get_link_jacobian(target_ids[0])
                   .block(3, 0, dim, robot_->n_q_dot);
    jacobian.block(0, 0, dim, robot_->n_floating) =
        Eigen::MatrixXd::Zero(dim, robot_->n_floating);
    jacobian_dot_q_dot =
        robot_->get_link_jacobian_dot_times_qdot(target_ids[0]).tail(3);
    break;
  }
  case Type::LINK_XYZ: {
    jacobian = robot_->get_link_jacobian(target_ids[0])
                   .block(3, 0, dim, robot_->n_q_dot);
    jacobian_dot_q_dot =
        robot_->get_link_jacobian_dot_times_qdot(target_ids[0]).tail(3);
    break;
  }
  case Type::ISOLATED_LINK_ORI: {
    jacobian = robot_->get_link_jacobian(target_ids[0])
                   .block(0, 0, dim, robot_->n_q_dot);
    jacobian.block(0, 0, dim, robot_->n_floating) =
        Eigen::MatrixXd::Zero(dim, robot_->n_floating);
    jacobian_dot_q_dot =
        robot_->get_link_jacobian_dot_times_qdot(target_ids[0]).head(3);
    break;
  }
  case Type::LINK_ORI: {
    jacobian = robot_->get_link_jacobian(target_ids[0])
                   .block(0, 0, dim, robot_->n_q_dot);
    jacobian_dot_q_dot =
        robot_->get_link_jacobian_dot_times_qdot(target_ids[0]).head(3);
    break;
  }
  case Type::ISOLATED_COM: {
    jacobian = robot_->get_com_lin_jacobian();
    jacobian.block(0, 0, dim, robot_->n_floating) =
        Eigen::MatrixXd::Zero(dim, robot_->n_floating);
    jacobian_dot_q_dot =
        robot_->get_com_lin_jacobian_dot() * robot_->get_q_dot();
    break;
  }
  case Type::COM: {
    jacobian = robot_->get_com_lin_jacobian();
    jacobian_dot_q_dot =
        robot_->get_com_lin_jacobian_dot() * robot_->get_q_dot();
    break;
  }
  default: { assert(false); }
  }
}
