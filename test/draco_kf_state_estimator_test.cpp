#include <gtest/gtest.h>

#include "configuration.hpp"
#include "pnc/robot_system/dart_robot_system.hpp"
#include "pnc/draco_pnc/draco_kf_state_estimator.hpp"

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

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}