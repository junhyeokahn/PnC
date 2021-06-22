#pragma once

#include <PnC/WBC/BasicTask.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

// Object to manage common trajectory primitives
class FloatingBaseTrajectoryManager {
public:
  FloatingBaseTrajectoryManager(Task *_com_task, Task *_base_ori_task,
                                RobotSystem *_robot);
  ~FloatingBaseTrajectoryManager(){};

  void initializeFloatingBaseTrajectory(const double _start_time,
                                        const double _duration,
                                        const Eigen::VectorXd &_target_com_pos);
  void initializeCoMSwaying(double _start_time, double _duration,
                            Eigen::VectorXd _dis);
  void initializeCoMSinusoid(double _start_time, double _amp, double _freq);
  void updateFloatingBaseDesired(const double current_time);
  void paramInitialization(const YAML::Node &node){};

  Task *com_task_;
  Task *base_ori_task_;

  Eigen::VectorXd com_pos_des_;
  Eigen::VectorXd com_vel_des_;
  Eigen::VectorXd com_acc_des_;

  Eigen::VectorXd dcm_pos_des_;
  Eigen::VectorXd dcm_vel_des_;
  Eigen::VectorXd dcm_acc_des_;

  Eigen::Quaternion<double> base_ori_quat_des_;
  Eigen::VectorXd base_ori_des_;
  Eigen::VectorXd base_ang_vel_des_;
  Eigen::VectorXd base_ang_acc_des_;

  // Updates the task desired values
  void updateDesired();

  double start_time_;
  double duration_;
  Eigen::VectorXd ini_com_pos_;
  Eigen::VectorXd ini_base_quat_;
  Eigen::VectorXd target_com_pos_;
  std::string base_id_;
  bool is_swaying;
  bool is_sinusoid;
  Eigen::VectorXd amp;
  Eigen::VectorXd freq;
  Eigen::VectorXd mid_point;

  RobotSystem *robot_;
};
