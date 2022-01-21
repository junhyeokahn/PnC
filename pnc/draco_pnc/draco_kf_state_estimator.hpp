#pragma once

/**
 * Linear KF State Estimation Approach
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
#include "draco_state_provider.hpp"
#include "pnc/state_estimator/MARGFilter.hpp"
#include "draco_interface.hpp"

class DracoKFStateEstimator : HumanoidStateEstimator {
public:
  DracoKFStateEstimator(RobotSystem *robot);
  ~DracoKFStateEstimator();

  void initialize(DracoSensorData *);
  void update(DracoSensorData *);

protected:
  DracoStateProvider *sp_;

  MARGFilter margFilter;

  Eigen::Isometry3d iso_base_joint_to_imu_;
  Eigen::Isometry3d iso_base_com_to_imu_;

  Eigen::Vector3d global_linear_offset_;
  Eigen::Vector3d prev_base_joint_pos_;
  Eigen::Vector3d prev_base_com_pos_;

  std::vector<SimpleMovingAverage> com_vel_filter_;
  std::vector<SimpleMovingAverage> imu_ang_vel_filter_;
  std::vector<SimpleMovingAverage> cam_filter_;

};
