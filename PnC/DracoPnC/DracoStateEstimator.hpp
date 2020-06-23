#pragma once

#include <Eigen/Dense>
#include <vector>

#include <Configuration.h>

class DracoStateProvider;
class RobotSystem;
class BasicAccumulation;
class DracoSensorData;
class filter;

class DracoStateEstimator {
 public:
  DracoStateEstimator(RobotSystem* robot);
  ~DracoStateEstimator();

  void initialization(DracoSensorData*);
  void update(DracoSensorData*);

 protected:
  double joint_velocity_filter_freq_;
  double angular_velocity_filter_freq_;

  double initial_height_;
  int fixed_foot_;
  Eigen::Vector3d foot_pos_;
  DracoStateProvider* sp_;
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
  filter* x_vel_est_;
  filter* y_vel_est_;
  filter* z_vel_est_;

  void _JointUpdate(DracoSensorData* data);
  void _ConfigurationAndModelUpdate();
  void _FootContactUpdate(DracoSensorData* data);
  void _UpdateDCM();
  void MapToTorso_(const Eigen::VectorXd& imu_acc,
                   const Eigen::VectorXd& imu_angvel,
                   std::vector<double>& torso_acc,
                   std::vector<double>& torso_angvel);

  // clamp helper function
  double clamp_value(double in, double min, double max);

  // compute alpha from break frequency
  double computeAlphaGivenBreakFrequency(double hz, double dt);
};
