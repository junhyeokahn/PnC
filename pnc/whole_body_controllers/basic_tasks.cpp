#include "pnc/whole_body_controllers/basic_tasks.hpp"

#include <assert.h>

#include "configuration.hpp"
#include "utils/util.hpp"

JointTask::JointTask(RobotSystem *_robot) : Task(_robot, _robot->n_a) {

  // DataManager *data_manager = DataManager::GetDataManager();
  // data_manager->RegisterData(&pos_des, VECT, "j_pos_d", dim);
  // data_manager->RegisterData(&pos, VECT, "j_pos", dim);
  // data_manager->RegisterData(&vel_des, VECT, "j_vel_d", dim);
  // data_manager->RegisterData(&vel, VECT, "jvel", dim);
  // data_manager->RegisterData(&w_hierarchy, DOUBLE, "j_w");

  util::PrettyConstructor(3, "Joint Task ");
}

void JointTask::update_cmd() {
  pos = robot_->joint_positions;
  vel = robot_->joint_velocities;
  pos_err = pos_des - pos;

  for (int i = 0; i < dim; ++i) {
    op_cmd[i] = acc_des[i] + kp[i] * pos_err[i] + kd[i] * (vel_des[i] - vel[i]);
  }
}

void JointTask::update_jacobian() {
  jacobian.block(0, robot_->n_floating, dim, robot_->n_q_dot) =
      Eigen::MatrixXd::Zero(dim, robot_->n_q_dot);
  jacobian_dot_q_dot = Eigen::VectorXd::Zero(dim);
}

SelectedJointTask::SelectedJointTask(RobotSystem *_robot,
                                     std::vector<std::string> _target_ids)
    : Task(_robot, _target_ids.size(), _target_ids) {

  // DataManager *data_manager = DataManager::GetDataManager();
  // data_manager->RegisterData(&pos_des, VECT, "sj_pos_d", dim);
  // data_manager->RegisterData(&pos, VECT, "sj_pos", dim);
  // data_manager->RegisterData(&vel_des, VECT, "sj_vel_d", dim);
  // data_manager->RegisterData(&vel, VECT, "sj_vel", dim);
  // data_manager->RegisterData(&w_hierarchy, DOUBLE, "sj_w");

  util::PrettyConstructor(3, "Selected Joint Task ");
}

void SelectedJointTask::update_cmd() {
  for (int i = 0; i < target_ids.size(); ++i) {
    pos[i] = robot_->joint_positions[robot_->get_joint_idx(target_ids[i])];
    vel[i] = robot_->joint_velocities[robot_->get_joint_idx(target_ids[i])];
  }
  pos_err = pos_des - pos;

  for (int i = 0; i < dim; ++i) {
    op_cmd[i] = acc_des[i] + kp[i] * pos_err[i] + kd[i] * (vel_des[i] - vel[i]);
  }
}

void SelectedJointTask::update_jacobian() {
  for (int i = 0; i < target_ids.size(); ++i) {
    jacobian(i, robot_->get_q_dot_idx(target_ids[i])) = 1.;
  }
  jacobian_dot_q_dot = Eigen::VectorXd::Zero(dim);
}

LinkPosTask::LinkPosTask(RobotSystem *_robot,
                         std::vector<std::string> _target_ids,
                         std::string _topic_name)
    : Task(_robot, 3 * _target_ids.size(), _target_ids) {
  assert(dim == 3 * _target_ids.size());

  if (_topic_name.empty()) {
    // Do not save
  } else {
    // DataManager *data_manager = DataManager::GetDataManager();
    // data_manager->RegisterData(&pos_des, VECT, _topic_name + "_pos_d", dim);
    // data_manager->RegisterData(&pos, VECT, _topic_name + "_pos", dim);
    // data_manager->RegisterData(&vel_des, VECT, _topic_name + "_vel_d", dim);
    // data_manager->RegisterData(&vel, VECT, _topic_name + "_vel", dim);
    // data_manager->RegisterData(&w_hierarchy, DOUBLE, _topic_name + "_pos_w");
  }
  util::PrettyConstructor(3, "Link Pos Task ");
}

void LinkPosTask::update_cmd() {

  for (int i = 0; i < target_ids.size(); ++i) {
    pos.segment(3 * i, 3) = robot_->get_link_iso(target_ids[i]).translation();
    vel.segment(3 * i, 3) = robot_->get_link_vel(target_ids[i]).tail(3);
  }
  pos_err = pos_des - pos;

  for (int i = 0; i < dim; ++i) {
    op_cmd[i] = acc_des[i] + kp[i] * pos_err[i] + kd[i] * (vel_des[i] - vel[i]);
  }
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
                         std::vector<std::string> _target_ids,
                         std::string _topic_name)
    : Task(_robot, 3 * _target_ids.size(), _target_ids) {
  assert(dim == 3 * _target_ids.size());
  pos_des = Eigen::VectorXd::Zero(4 * _target_ids.size());
  pos = Eigen::VectorXd::Zero(4 * _target_ids.size());

  if (_topic_name.empty()) {
    // Do not save
  } else {
    // DataManager *data_manager = DataManager::GetDataManager();
    // data_manager->RegisterData(&pos_des, VECT, _topic_name + "_ori_d", dim);
    // data_manager->RegisterData(&pos, VECT, _topic_name + "_ori", dim);
    // data_manager->RegisterData(&vel_des, VECT, _topic_name + "_angvel_d",
    // dim); data_manager->RegisterData(&vel, VECT, _topic_name + "_angvel",
    // dim); data_manager->RegisterData(&w_hierarchy, DOUBLE, _topic_name +
    // "_ori_w");
  }

  util::PrettyConstructor(3, "Link Ori Task ");
}

void LinkOriTask::update_cmd() {

  for (int i = 0; i < target_ids.size(); ++i) {
    Eigen::Quaternion<double> quat_des(pos_des[4 * i + 0], pos_des[4 * i + 1],
                                       pos_des[4 * i + 2], pos_des[4 * i + 3]);
    Eigen::Quaternion<double> quat(
        robot_->get_link_iso(target_ids[i]).linear());
    util::AvoidQuatJump(quat_des, quat);
    Eigen::Quaternion<double> quat_err = quat_des * quat.inverse();
    Eigen::Vector3d ori_err = util::QuatToExp(quat_err);
    pos.segment(4 * i, 4) =
        Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z());
    pos_err.segment(3 * i, 3) = ori_err;
    vel.segment(3 * i, 3) = robot_->get_link_vel(target_ids[i]).head(3);
  }

  for (int i = 0; i < dim; ++i) {
    op_cmd[i] = acc_des[i] + kp[i] * pos_err[i] + kd[i] * (vel_des[i] - vel[i]);
  }
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

  // DataManager *data_manager = DataManager::GetDataManager();
  // data_manager->RegisterData(&pos_des, VECT, "com_pos_d", dim);
  // data_manager->RegisterData(&pos, VECT, "com_pos", dim);
  // data_manager->RegisterData(&vel_des, VECT, "com_vel_d", dim);
  // data_manager->RegisterData(&vel, VECT, "com_vel", dim);
  // data_manager->RegisterData(&w_hierarchy, DOUBLE, "com_w");

  util::PrettyConstructor(3, "Center Of Mass Task ");
}

void CenterOfMassTask::update_cmd() {

  pos = robot_->get_com_pos();
  pos_err = pos_des - pos;
  vel = robot_->get_com_lin_vel();

  for (int i = 0; i < dim; ++i) {
    op_cmd[i] = acc_des[i] + kp[i] * pos_err[i] + kd[i] * (vel_des[i] - vel[i]);
  }
}

void CenterOfMassTask::update_jacobian() {
  jacobian = robot_->get_com_lin_jacobian();
  jacobian_dot_q_dot = robot_->get_com_lin_jacobian_dot() * robot_->get_q_dot();
}
