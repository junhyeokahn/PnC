#pragma once

#include <Configuration.h>
#include <Eigen/Dense>
#include <vector>

class A1StateProvider;
class RobotSystem;
class A1SensorData;
class BasicAccumulation;
class filter;
class AverageFilter;

class A1StateEstimator {
 public:
  A1StateEstimator(RobotSystem* robot);
  ~A1StateEstimator();

  void Initialization(A1SensorData*);
  void Update(A1SensorData*);

 protected:
  double joint_velocity_filter_freq_;
  double angular_velocity_filter_freq_;

  double initial_height_;
  int front_fixed_foot_;
  int rear_fixed_foot_;
  Eigen::Vector3d front_foot_pos_;
  Eigen::Vector3d rear_foot_pos;

  A1StateProvider* sp_;
  RobotSystem* robot_;

  Eigen::VectorXd curr_config_;
  Eigen::Vector3d global_linear_offset_;
  Eigen::VectorXd prev_config_;

  Eigen::VectorXd curr_qdot_;
  Eigen::VectorXd prev_qdot_;
  Eigen::Vector3d global_body_euler_zyx_;
  Eigen::Quaternion<double> global_body_quat_;
  Eigen::Vector3d global_body_euler_zyx_dot_;
  Eigen::Vector3d prev_body_euler_zyx_dot_;
  Eigen::VectorXd virtual_q_;
  Eigen::VectorXd virtual_qdot_;

  BasicAccumulation* ori_est_;
  AverageFilter* x_vel_est_;
  // AverageFilter* y_vel_est_;
  // AverageFilter* z_vel_est_;
  // filter* x_vel_est_;
  // filter* y_vel_est_;
  // filter* z_vel_est_;

  void _JointUpdate(A1SensorData* data);
  void _ConfigurationAndModelUpdate();
  void _FootContactUpdate(A1SensorData* data);
  void MapToTorso_(const Eigen::VectorXd& imu_acc,
                   const Eigen::VectorXd& imu_angvel,
                   std::vector<double>& trunk_acc,
                   std::vector<double>& trunk_angvel);

  double clamp_value(double in, double min, double max);

  double computeAlphaGivenBreakFrequency(double hz, double dt);
};
