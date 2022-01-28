#include <gtest/gtest.h>

#include "configuration.hpp"
#include "pnc/robot_system/dart_robot_system.hpp"
#include "pnc/state_estimator/MARGFilter.hpp"

#include "pnc/state_estimator/FloatingBaseSystemModel.hpp"
#include "pnc/state_estimator/PoseMeasurementModel.hpp"
#include "third_party/kalman_filters/ExtendedKalmanFilter.hpp"

const double error_tolerance = 1.0e-3;

TEST(DracoKFStateEstimatorTest, standingStraight)
{
  // Note:
  // the magnetometer measurement obtained for Austin, TX in microTeslas
  // (https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm)
  Eigen::Vector3f gyro(0.0, 0.0, 0.0);
  Eigen::Vector3f accelerometer(0.0, 0.0, 9.8);
  Eigen::Vector3f magnetometer(24.16, 1.45, 39.98);

  Eigen::Matrix3d expectedRotMatrix = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d estimatedMARGRotMatrix, estimatedIMURotMatrix;

  MARGFilter *margFilter = new MARGFilter();
  MARGFilter *imuFilter = new MARGFilter();
  margFilter->filterUpdate(gyro(0), gyro(1), gyro(2),
                          accelerometer(0), accelerometer(1), accelerometer(2),
                          magnetometer(0), magnetometer(1), magnetometer(2));
  estimatedMARGRotMatrix = margFilter->getBaseRotation();
  
  imuFilter->filterUpdate(gyro(0), gyro(1), gyro(2),
                          accelerometer(0), accelerometer(1), accelerometer(2));
  estimatedIMURotMatrix = imuFilter->getBaseRotation();
  
  // check error using matrix 2-norm
  Eigen::Matrix3d rotation_error_marg = expectedRotMatrix - estimatedMARGRotMatrix;
  Eigen::Matrix3d rotation_error_imu = expectedRotMatrix - estimatedIMURotMatrix;
//  std::cout << "Error in MARG rotation matrix: " << std::endl << rotation_error_marg << std::endl;
//  std::cout << "MARG error norm: " << rotation_error_marg.norm() << std::endl;
//  std::cout << "Error in IMU rotation matrix: " << std::endl << rotation_error_imu << std::endl;
//  std::cout << "IMU error norm: " << rotation_error_imu.norm() << std::endl;
  ASSERT_LE((rotation_error_marg).norm(), error_tolerance);
  ASSERT_LE((rotation_error_imu).norm(), error_tolerance);
}

TEST(DracoKFStateEstimatorTest, linearKFZeroNoise)
{
  // parameters used for testing
  Eigen::Matrix3d rot_world_to_base = Eigen::Matrix3d::Identity();
  Eigen::Vector3d gravity, imu_accel;
  gravity << 0.0, 0.0, -9.81;
  imu_accel << 0.0, 0.0, 9.81;

  // stuff needed for the kalman filter
  State x, x_hat;
  Control accelerometer_input;
  FloatingBaseSystemModel system_model;
  PoseMeasurement base_estimate;
  PoseMeasurementModel base_pose_model;
  Kalman::ExtendedKalmanFilter<State> kalman_filter;

  // initialize estimated state x_hat = [0_pos_b, 0_vel_b, 0_pos_LF, 0_pos_RF]
  x_hat.setZero();
  x_hat.base_pos_z() = 1.0;
  x_hat.lfoot_pos_y() = 0.2;
  x_hat.rfoot_pos_y() = -0.2;
  x = x_hat;

  // initialize estimated state system dynamics (matrices A and B)
  system_model.initialize(deltat);

  // initialize output matrix H in z_hat = H * x_hat
  base_pose_model.initialize(gravity);

  // set initial state of the system
  kalman_filter.init(x_hat);

  // assume robot is static and predict new state and covariance matrix
  base_pose_model.packAccelerationInput(rot_world_to_base, imu_accel, accelerometer_input);
  x_hat = kalman_filter.predict(system_model, accelerometer_input);

  // compute innovation covariance, kalman gain, and update state and covariance
  base_estimate.base_pose_lfoot_y() = -0.2;
  base_estimate.base_pose_rfoot_y() = 0.2;
  base_estimate.base_pose_lfoot_z() = 1.0;
  base_estimate.base_pose_rfoot_z() = 1.0;
  x_hat = kalman_filter.update(base_pose_model, base_estimate);

  ASSERT_LE((x - x_hat).norm() , error_tolerance);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}