#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Utils/IO/IOUtilities.hpp>
#include <iostream>
#include <stdio.h>

namespace myUtils {

// =============================================================================
// Matrix Utils
// =============================================================================
Eigen::MatrixXd hStack(const Eigen::MatrixXd &a_, const Eigen::MatrixXd &b_);
Eigen::MatrixXd vStack(const Eigen::MatrixXd &a_, const Eigen::MatrixXd &b_);
Eigen::MatrixXd vStack(const Eigen::VectorXd &a_, const Eigen::VectorXd &b_);
Eigen::MatrixXd block_diag(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b);
Eigen::MatrixXd deleteRow(const Eigen::MatrixXd &a_, int row);

// =============================================================================
// Simple Trajectory Generator
// =============================================================================
double smooth_changing(double ini, double end, double moving_duration,
                       double curr_time);
double smooth_changing_vel(double ini, double end, double moving_duration,
                           double curr_time);
double smooth_changing_acc(double ini, double end, double moving_duration,
                           double curr_time);
void getSinusoidTrajectory(double initTime_, const Eigen::VectorXd &midPoint_,
                           const Eigen::VectorXd &amp_,
                           const Eigen::VectorXd &freq_, double evalTime_,
                           Eigen::VectorXd &p_, Eigen::VectorXd &v_,
                           Eigen::VectorXd &a_);
double smoothing(double ini, double fin, double rat);

// =============================================================================
// ETC
// =============================================================================

double computeAlphaGivenBreakFrequency(double hz, double dt);

double bind_half_pi(double);

bool isEqual(const Eigen::VectorXd a, const Eigen::VectorXd b,
             const double threshold = 0.00001);
double CropValue(double value, double min, double max, std::string source);
double CropValue(double value, double min, double max);

Eigen::VectorXd CropVector(Eigen::VectorXd value, Eigen::VectorXd min,
                           Eigen::VectorXd max, std::string source);

Eigen::MatrixXd CropMatrix(Eigen::MatrixXd value, Eigen::MatrixXd min,
                           Eigen::MatrixXd max, std::string source);

bool isInBoundingBox(const Eigen::VectorXd &val, const Eigen::VectorXd &lb,
                     const Eigen::VectorXd &ub);

Eigen::MatrixXd GetRelativeMatrix(const Eigen::MatrixXd value,
                                  const Eigen::MatrixXd min,
                                  const Eigen::MatrixXd max);

Eigen::VectorXd GetRelativeVector(const Eigen::VectorXd value,
                                  const Eigen::VectorXd min,
                                  const Eigen::VectorXd max);

Eigen::VectorXd eulerIntegration(const Eigen::VectorXd &x,
                                 const Eigen::VectorXd &xdot, double dt);

Eigen::VectorXd doubleIntegration(const Eigen::VectorXd &q,
                                  const Eigen::VectorXd &alpha,
                                  const Eigen::VectorXd &alphad, double dt);

Eigen::Matrix3d VecToso3(const Eigen::Vector3d &omg);
Eigen::MatrixXd Adjoint(const Eigen::MatrixXd &R, const Eigen::Vector3d &p);

Eigen::Vector3d quat_to_exp(const Eigen::Quaternion<double> &quat);
Eigen::Quaternion<double> exp_to_quat(const Eigen::Vector3d &exp);

double QuatToYaw(const Eigen::Quaternion<double> q);

// Euler ZYX
//     Represents either:
//     extrinsic XYZ rotations: Fixed-frame roll, then fixed-frame pitch, then
//     fixed-frame yaw.
//     or intrinsic ZYX rotations: Body-frame yaw, body-frame pitch, then
//     body-frame roll
//
//     The equation is similar, but the values for fixed and body frame
//     rotations are different.
// World Orientation is R = Rz*Ry*Rx
Eigen::Quaterniond EulerZYXtoQuat(const double roll, const double pitch,
                                  const double yaw);

// Quaternion to Euler ZYX
Eigen::Vector3d QuatToEulerZYX(const Eigen::Quaterniond &quat_in);

// ZYX extrinsic rotation rates to world angular velocity
// angular vel = [wx, wy, wz]
Eigen::Vector3d EulerZYXRatestoAngVel(const double roll, const double pitch,
                                      const double yaw, const double roll_rate,
                                      const double pitch_rate,
                                      const double yaw_rate);

void avoid_quat_jump(const Eigen::Quaternion<double> &des_ori,
                     Eigen::Quaternion<double> &act_ori);
} // namespace myUtils
