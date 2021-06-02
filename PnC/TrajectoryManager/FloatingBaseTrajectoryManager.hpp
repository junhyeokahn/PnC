#pragma once

#include <PnC/TrajectoryManager/TrajectoryManagerBase.hpp>
#include <PnC/WBC/BasicTask.hpp>
#include <PnC/ConvexMPC/ConvexMPC.hpp>
#include <Eigen/Dense>


// Object to manage common trajectory primitives
class FloatingBaseTrajectoryManager : public TrajectoryManagerBase {
 public:
  FloatingBaseTrajectoryManager(Task* _com_task, Task* _base_ori_task,
                                RobotSystem* _robot);
  ~FloatingBaseTrajectoryManager(){};

  void initializeFloatingBaseTrajectory(const double _start_time,
                                        const double _end_time,
                                        const Eigen::VectorXd& _target_com_pos);
  void initializeCoMSwaying(double _start_time, double _duration,
                            Eigen::VectorXd _dis);
  void initializeCoMSinusoid(double _start_time, double _amp, double _freq);
  void updateFloatingBaseDesired(const double current_time);
  void updateFloatingBaseWalkingDesired(Eigen::VectorXd curr_com_pos,
                                        Eigen::VectorXd x_y_yaw_vel_des);
  void paramInitialization(const YAML::Node& node){};


  Task* com_task_;
  Task* base_ori_task_;

  Eigen::VectorXd com_pos_des_;
  Eigen::VectorXd com_vel_des_;
  Eigen::VectorXd prev_com_vel_des_;
  Eigen::VectorXd com_acc_des_;

  Eigen::Quaternion<double> base_ori_quat_des_;
  Eigen::VectorXd base_ori_des_;
  Eigen::VectorXd base_ang_vel_des_;
  Eigen::VectorXd prev_base_ang_vel_des_;
  Eigen::VectorXd base_ang_acc_des_;

  // Updates the task desired values
  void updateDesired();

  double start_time_;
  double duration_;
  Eigen::VectorXd ini_com_pos_;
  Eigen::VectorXd ini_base_quat_;
  Eigen::VectorXd target_com_pos_;
  int base_id_;
  bool is_swaying;
  bool is_sinusoid;
  Eigen::VectorXd amp;
  Eigen::VectorXd freq;
  Eigen::VectorXd mid_point;

  double integrated_yaw_des;
  private:
  Eigen::Vector3d toRPY(Eigen::Quaternion<double> quat);
};
