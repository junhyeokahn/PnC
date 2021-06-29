#include <assert.h>

#include <Configuration.hpp>
#include <PnC/WBC/BasicTask.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

JointTask::JointTask(RobotSystem *_robot) : Task(_robot, _robot->n_a) {

  DataManager *data_manager = DataManager::GetDataManager();
  data_manager->RegisterData(&pos_des_, VECT, "t_j_pos_d", dim);
  data_manager->RegisterData(&pos_, VECT, "t_j_pos", dim);
  data_manager->RegisterData(&vel_des_, VECT, "t_j_vel_d", dim);
  data_manager->RegisterData(&vel_, VECT, "t_jvel", dim);
  data_manager->RegisterData(&w_hierarchy, DOUBLE, "t_j_w");

  myUtils::pretty_constructor(3, "Joint Task ");
}

void JointTask::update_cmd() {
  pos_ = robot_->joint_positions;
  vel_ = robot_->joint_velocities;
  pos_err = pos_des_ - pos_;

  for (int i = 0; i < dim; ++i) {
    op_cmd[i] =
        acc_des_[i] + kp[i] * pos_err[i] + kd[i] * (vel_des_[i] - vel_[i]);
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

  DataManager *data_manager = DataManager::GetDataManager();
  data_manager->RegisterData(&pos_des_, VECT, "sj_pos_d", dim);
  data_manager->RegisterData(&pos_, VECT, "sj_pos", dim);
  data_manager->RegisterData(&vel_des_, VECT, "sjvel_d", dim);
  data_manager->RegisterData(&vel_, VECT, "sj_vel", dim);
  data_manager->RegisterData(&w_hierarchy, DOUBLE, "sj_w");

  myUtils::pretty_constructor(3, "Selected Joint Task ");
}

void SelectedJointTask::update_cmd() {
  for (int i = 0; i < target_ids.size(); ++i) {
    pos_[i] = robot_->joint_positions[robot_->get_joint_idx(target_ids[i])];
    vel_[i] = robot_->joint_velocities[robot_->get_joint_idx(target_ids[i])];
  }
  pos_err = pos_des_ - pos_;

  for (int i = 0; i < dim; ++i) {
    op_cmd[i] =
        acc_des_[i] + kp[i] * pos_err[i] + kd[i] * (vel_des_[i] - vel_[i]);
  }
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

  DataManager *data_manager = DataManager::GetDataManager();
  data_manager->RegisterData(&pos_des_, VECT, _target_ids[0] + "_pos_des", dim);
  data_manager->RegisterData(&pos_, VECT, target_ids[0] + "_pos", dim);
  data_manager->RegisterData(&vel_des_, VECT, target_ids[0] + "_vel_des", dim);
  data_manager->RegisterData(&vel_, VECT, target_ids[0] + "_vel", dim);
  data_manager->RegisterData(&w_hierarchy, DOUBLE, target_ids[0] + "_pos_w");

  myUtils::pretty_constructor(3, "Link Pos Task ");
}

void LinkPosTask::update_cmd() {

  for (int i = 0; i < target_ids.size(); ++i) {
    pos_.segment(3 * i, 3) = robot_->get_link_iso(target_ids[i]).translation();
    vel_.segment(3 * i, 3) = robot_->get_link_vel(target_ids[i]).tail(3);
  }
  pos_err = pos_des_ - pos_;

  for (int i = 0; i < dim; ++i) {
    op_cmd[i] =
        acc_des_[i] + kp[i] * pos_err[i] + kd[i] * (vel_des_[i] - vel_[i]);
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
                         std::vector<std::string> _target_ids)
    : Task(_robot, 3 * _target_ids.size(), _target_ids) {
  assert(dim == 3 * _target_ids.size());
  pos_des_ = Eigen::VectorXd::Zero(4 * _target_ids.size());
  pos_ = Eigen::VectorXd::Zero(4 * _target_ids.size());

  DataManager *data_manager = DataManager::GetDataManager();
  data_manager->RegisterData(&pos_des_, VECT, _target_ids[0] + "_ori_d",
                             dim + 1);
  data_manager->RegisterData(&pos_, VECT, target_ids[0] + "_ori", dim + 1);
  data_manager->RegisterData(&vel_des_, VECT, target_ids[0] + "_angvel_d", dim);
  data_manager->RegisterData(&vel_, VECT, target_ids[0] + "_angvel", dim);
  data_manager->RegisterData(&w_hierarchy, DOUBLE, target_ids[0] + "_ori_w");

  myUtils::pretty_constructor(3, "Link Ori Task ");
}

void LinkOriTask::update_cmd() {

  for (int i = 0; i < target_ids.size(); ++i) {
    Eigen::Quaternion<double> quat_des(pos_des_[4 * i + 0], pos_des_[4 * i + 1],
                                       pos_des_[4 * i + 2],
                                       pos_des_[4 * i + 3]);
    Eigen::Quaternion<double> quat(
        robot_->get_link_iso(target_ids[i]).linear());
    myUtils::avoid_quat_jump(quat_des, quat);
    Eigen::Quaternion<double> quat_err = quat_des * quat.inverse();
    Eigen::Vector3d ori_err = myUtils::quat_to_exp(quat_err);
    pos_.segment(4 * i, 4) =
        Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z());
    pos_err.segment(3 * i, 3) = ori_err;
    vel_.segment(3 * i, 3) = robot_->get_link_vel(target_ids[i]).head(3);
  }

  for (int i = 0; i < dim; ++i) {
    op_cmd[i] =
        acc_des_[i] + kp[i] * pos_err[i] + kd[i] * (vel_des_[i] - vel_[i]);
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

  DataManager *data_manager = DataManager::GetDataManager();
  data_manager->RegisterData(&pos_des_, VECT, "com_pos_d", dim);
  data_manager->RegisterData(&pos_, VECT, "com_pos", dim);
  data_manager->RegisterData(&vel_des_, VECT, "com_vel_d", dim);
  data_manager->RegisterData(&vel_, VECT, "com_vel", dim);
  data_manager->RegisterData(&w_hierarchy, DOUBLE, "com_w");

  myUtils::pretty_constructor(3, "Center Of Mass Task ");
}

void CenterOfMassTask::update_cmd() {

  pos_ = robot_->get_com_pos();
  pos_err = pos_des_ - pos_;
  vel_ = robot_->get_com_lin_vel();

  for (int i = 0; i < dim; ++i) {
    op_cmd[i] =
        acc_des_[i] + kp[i] * pos_err[i] + kd[i] * (vel_des_[i] - vel_[i]);
  }
}

void CenterOfMassTask::update_jacobian() {
  jacobian = robot_->get_com_lin_jacobian();
  jacobian_dot_q_dot = robot_->get_com_lin_jacobian_dot() * robot_->get_q_dot();
}
