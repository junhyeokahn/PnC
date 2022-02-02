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

#include <vector>
#include <Eigen/Dense>

#include "configuration.hpp"
#include "pnc/filters/digital_filters.hpp"
#include "pnc/state_estimator/humanoid_state_estimator.hpp"
#include "utils/util.hpp"
#include "pnc/draco_pnc/draco_state_provider.hpp"
#include "pnc/state_estimator/MARGFilter.hpp"
#include "pnc/draco_pnc/draco_interface.hpp"

// kalman filter files
#include "pnc/state_estimator/FloatingBaseSystemModel.hpp"
#include "pnc/state_estimator/PoseMeasurementModel.hpp"
#include "third_party/kalman_filters/ExtendedKalmanFilter.hpp"

class DracoSensorData;

class DracoKFStateEstimator {
public:
  DracoKFStateEstimator(RobotSystem *robot);
  ~DracoKFStateEstimator();

  void initialize(DracoSensorData *);
  void update(DracoSensorData *);

protected:
  RobotSystem *robot_;
  DracoStateProvider *sp_;

  // stuff needed for the kalman filter
  State x_hat_;
  Control accelerometer_input_;
  FloatingBaseSystemModel system_model_;
  PoseMeasurement base_estimate_;
  PoseMeasurementModel base_pose_model_;
  Kalman::ExtendedKalmanFilter<State> kalman_filter_;
  MARGFilter margFilter_;

  Eigen::Matrix3d rot_world_to_base;

    bool b_first_visit_;
};
