#pragma once

#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoTask/CoMxyz.hpp>
#include <PnC/DracoPnC/DracoTask/DCMTask.hpp>
#include <PnC/DracoPnC/DracoTask/FootLocalRyRzTask.hpp>
#include <PnC/DracoPnC/DracoTask/Footxyz.hpp>
#include <PnC/TaskAndForceContainer.hpp>
#include <PnC/WBC/BasicContactSpec.hpp>
#include <PnC/WBC/BasicTask.hpp>

// Object which publicly contains all the tasks, contacts and reaction forces
class DracoTaskAndForceContainer : public TaskAndForceContainer {
 public:
  DracoTaskAndForceContainer(RobotSystem* _robot);
  ~DracoTaskAndForceContainer();
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
  Task* dcm_task_;
  Task* joint_task_;
  Task* base_ori_task_;
  Task* rfoot_center_pos_task_;
  Task* lfoot_center_pos_task_;
  Task* rfoot_center_ori_task_;
  Task* lfoot_center_ori_task_;

  // -------------------------------------------------------
  // Contact Member variables
  // -------------------------------------------------------
  // ContactSpec* rfoot_front_contact_;
  // ContactSpec* rfoot_back_contact_;
  // ContactSpec* lfoot_front_contact_;
  // ContactSpec* lfoot_back_contact_;
  ContactSpec* rfoot_contact_;
  ContactSpec* lfoot_contact_;
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
  double w_task_foot_ori_;

  // Task Gains
  Eigen::VectorXd kp_joint_;
  Eigen::VectorXd kd_joint_;
  Eigen::VectorXd kp_com_;
  Eigen::VectorXd kd_com_;
  Eigen::VectorXd kp_base_ori_;
  Eigen::VectorXd kd_base_ori_;
  Eigen::VectorXd kp_foot_pos_;
  Eigen::VectorXd kd_foot_pos_;
  Eigen::VectorXd kp_foot_ori_;
  Eigen::VectorXd kd_foot_ori_;
};
