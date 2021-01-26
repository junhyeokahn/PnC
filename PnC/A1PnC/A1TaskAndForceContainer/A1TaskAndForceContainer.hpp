#pragma once

#include <PnC/DracoPnC/A1Definition.hpp>
#include <PnC/TaskAndForceContainer.hpp>
#include <PnC/WBC/BasicContactSpec.hpp>
#include <PnC/WBC/BasicTask.hpp>

// Object which publicly contains all the tasks, contacts and reaction forces
class A1TaskAndForceContainer : public TaskAndForceContainer {
 public:
  A1TaskAndForceContainer(RobotSystem* _robot);
  ~A1TaskAndForceContainer();
  void paramInitialization(const YAML::Node& node);

 protected:
  void _InitializeTasks();
  void _InitializeContacts();
  void _DeleteTasks();
  void _DeleteContacts();

 public:
  // -------------------------------------------------------
  // Task Member variables
  // -------------------------------------------------------
  Task* com_task_;
  Task* joint_task_;
  Task* base_ori_task_;
  Task* frfoot_pos_task_;
  Task* flfoot_pos_task_;
  Task* rrfoot_pos_task_;
  Task* rlfoot_pos_task_;

  // -------------------------------------------------------
  // Contact Member variables
  // -------------------------------------------------------
  ContactSpec* frfoot_contact_;
  ContactSpec* flfoot_contact_;
  ContactSpec* rrfoot_contact_;
  ContactSpec* rlfoot_contact_;
  int dim_contact_;
  double max_z_;
  Eigen::VectorXd Fd_des_;

  // -------------------------------------------------------
  // Parameters
  // -------------------------------------------------------
  // Task Hierarchy Weights
  Eigen::VectorXd w_task_hierarchy_;
  double w_task_com_;
  double w_task_base_ori_;
  double w_task_foot_pos_;
  // double w_task_foot_ori_;

  // Task Gains
  Eigen::VectorXd kp_joint_;
  Eigen::VectorXd kd_joint_;
  Eigen::VectorXd kp_com_;
  Eigen::VectorXd kd_com_;
  Eigen::VectorXd kp_base_ori_;
  Eigen::VectorXd kd_base_ori_;
  Eigen::VectorXd kp_foot_pos_;
  Eigen::VectorXd kd_foot_pos_;
};
