#pragma once

#include <stdio.h>
#include <Eigen/Dense>
#include <iostream>

namespace myUtils {

// =============================================================================
// Matrix Utils
// =============================================================================
Eigen::MatrixXd hStack(const Eigen::MatrixXd& a_, const Eigen::MatrixXd& b_);
Eigen::MatrixXd vStack(const Eigen::MatrixXd& a_, const Eigen::MatrixXd& b_);
Eigen::MatrixXd vStack(const Eigen::VectorXd& a_, const Eigen::VectorXd& b_);
Eigen::MatrixXd deleteRow(const Eigen::MatrixXd& a_, int row);

// =============================================================================
// Simple Trajectory Generator
// =============================================================================
double smooth_changing(double ini, double end, double moving_duration,
                       double curr_time);
double smooth_changing_vel(double ini, double end, double moving_duration,
                           double curr_time);
double smooth_changing_acc(double ini, double end, double moving_duration,
                           double curr_time);
void getSinusoidTrajectory(double initTime_, const Eigen::VectorXd& midPoint_,
                           const Eigen::VectorXd& amp_,
                           const Eigen::VectorXd& freq_, double evalTime_,
                           Eigen::VectorXd& p_, Eigen::VectorXd& v_,
                           Eigen::VectorXd& a_);

// =============================================================================
// ETC
// =============================================================================
double bind_half_pi(double);
bool isEqual(const Eigen::VectorXd a, const Eigen::VectorXd b,
             const double threshold = 0.00001);
double cropValue(double value, double min, double max, std::string source);
bool isInBoundingBox(const Eigen::VectorXd& lb, const Eigen::VectorXd& val,
                     const Eigen::VectorXd& ub);
Eigen::VectorXd eulerIntegration(const Eigen::VectorXd& x,
                                 const Eigen::VectorXd& xdot, double dt);
Eigen::VectorXd doubleIntegration(const Eigen::VectorXd& q,
                                  const Eigen::VectorXd& alpha,
                                  const Eigen::VectorXd& alphad, double dt);
}  // namespace myUtils
