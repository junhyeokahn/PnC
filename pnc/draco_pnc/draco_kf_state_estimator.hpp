#pragma once

/**
 * Linear KF State Estimator
 *
 * Approach taken from second method in:
 *
 * Flayols, T., Del Prete, A., Wensing, P., Mifsud, A., Benallegue,
 * M. and Stasse, O., 2017, November. Experimental evaluation of simple
 * estimators for humanoid robots. In 2017 IEEE-RAS 17th International
 * Conference on Humanoid Robotics (Humanoids) (pp. 889-895). IEEE.
 */

#include <Eigen/Dense>
#include <vector>

#include "configuration.hpp"
#include "pnc/draco_pnc/draco_interface.hpp"
#include "pnc/draco_pnc/draco_state_provider.hpp"
#include "pnc/filters/digital_filters.hpp"
#include "pnc/state_estimator/MARGFilter.hpp"
#include "pnc/state_estimator/humanoid_state_estimator.hpp"
#include "utils/util.hpp"

// kalman filter files
#include "pnc/state_estimator/FloatingBaseSystemModel.hpp"
#include "pnc/state_estimator/PoseMeasurementModel.hpp"
#include "third_party/kalman_filters/ExtendedKalmanFilter.hpp"

class DracoSensorData;

class DracoKFStateEstimator {
public:
  enum SupportState { LEFT, RIGHT, DOUBLE };

  DracoKFStateEstimator(RobotSystem *robot);
  ~DracoKFStateEstimator();

  void initialize(DracoSensorData *);
  void update(DracoSensorData *);

  void ComputeDCM();

private:
  void updateSupportState(DracoStateProvider *sp, SupportState &support_state);
  Eigen::Matrix3d compute_world_to_base_rot(DracoSensorData *data,
                                            Eigen::Matrix3d rot_world_to_imu,
                                            bool use_marg_filter);

protected:
  RobotSystem *robot_;
  DracoStateProvider *sp_;
  SupportState current_support_state_;
  SupportState prev_support_state_;
  Eigen::Vector3d prev_base_com_pos_;
  Eigen::Vector3d foot_pos_from_base_pre_transition;
  Eigen::Vector3d foot_pos_from_base_post_transition;

  Eigen::Isometry3d iso_imu_to_base_com_;
  Eigen::Vector3d global_linear_offset_;

  // stuff needed for the kalman filter
  State x_hat_;
  Control accelerometer_input_;
  FloatingBaseSystemModel system_model_;
  PoseMeasurement base_estimate_;
  PoseMeasurementModel base_pose_model_;
  Kalman::ExtendedKalmanFilter<State> kalman_filter_;
  MARGFilter margFilter_;
  Eigen::Vector3d base_acceleration_;

  Eigen::Matrix3d rot_world_to_base;

  std::vector<SimpleMovingAverage> com_vel_filter_;
  std::vector<SimpleMovingAverage> cam_filter_;
  std::vector<SimpleMovingAverage> base_accel_filter_;
  std::vector<SimpleMovingAverage> imu_ang_vel_filter_;
  ExponentialMovingAverageFilter *base_accel_filt_;

  bool b_first_visit_;
  bool b_skip_prediction;
  bool b_use_marg_filter;
  bool b_request_offset_reset;
};
