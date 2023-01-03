#include "min_jerk_curve.hpp"

MinJerkCurve::MinJerkCurve() { Initialization(); }

MinJerkCurve::MinJerkCurve(const Eigen::Vector3d &init,
                           const Eigen::Vector3d &end, const double &time_start,
                           const double &time_end) {
  Initialization();
  SetParams(init, end, time_start, time_end);
}

// Destructor
MinJerkCurve::~MinJerkCurve() {}

void MinJerkCurve::Initialization() {
  // Initialize to the corrrect sizes
  C_mat = Eigen::MatrixXd::Zero(6, 6);
  C_mat_inv = Eigen::MatrixXd::Zero(6, 6);
  a_coeffs = Eigen::VectorXd::Zero(6);
  bound_cond = Eigen::VectorXd::Zero(6);

  init_cond = Eigen::VectorXd::Zero(3);
  end_cond = Eigen::VectorXd::Zero(3);
  to = 0.0;
  tf = 1.0;
}

void MinJerkCurve::SetParams(const Eigen::Vector3d &init,
                             const Eigen::Vector3d &end,
                             const double &time_start, const double &time_end) {
  // Set the Parameters
  init_cond = init;
  end_cond = end;
  bound_cond.head(3) = init_cond;
  bound_cond.tail(3) = end_cond;
  to = time_start;
  tf = time_end;

  // Construct C matrix
  C_mat(0, 0) = 1.0;
  C_mat(0, 1) = to;
  C_mat(0, 2) = std::pow(to, 2);
  C_mat(0, 3) = std::pow(to, 3);
  C_mat(0, 4) = std::pow(to, 4);
  C_mat(0, 5) = std::pow(to, 5);
  C_mat(1, 1) = 1.0;
  C_mat(1, 2) = 2.0 * to;
  C_mat(1, 3) = 3.0 * std::pow(to, 2);
  C_mat(1, 4) = 4.0 * std::pow(to, 3);
  C_mat(1, 5) = 5.0 * std::pow(to, 4);
  C_mat(2, 2) = 2.0;
  C_mat(2, 3) = 6.0 * to;
  C_mat(2, 4) = 12.0 * std::pow(to, 2);
  C_mat(2, 5) = 20.0 * std::pow(to, 3);
  C_mat(3, 0) = 1.0;
  C_mat(3, 1) = tf;
  C_mat(3, 2) = std::pow(tf, 2);
  C_mat(3, 3) = std::pow(tf, 3);
  C_mat(3, 4) = std::pow(tf, 4);
  C_mat(3, 5) = std::pow(tf, 5);
  C_mat(4, 1) = 1.0;
  C_mat(4, 2) = 2.0 * tf;
  C_mat(4, 3) = 3.0 * std::pow(tf, 2);
  C_mat(4, 4) = 4.0 * std::pow(tf, 3);
  C_mat(4, 5) = 5.0 * std::pow(tf, 4);
  C_mat(5, 2) = 2.0;
  C_mat(5, 3) = 6.0 * tf;
  C_mat(5, 4) = 12.0 * std::pow(tf, 2);
  C_mat(5, 5) = 20.0 * std::pow(tf, 3);

  // Solve for the coefficients
  C_mat_inv = C_mat.inverse();
  a_coeffs = C_mat_inv * bound_cond;
}

double MinJerkCurve::evaluate(const double &time) {
  s_ = clamp(time, to, tf);
  return a_coeffs[0] + a_coeffs[1] * s_ + a_coeffs[2] * std::pow(s_, 2) +
        a_coeffs[3] * std::pow(s_, 3) + a_coeffs[4] * std::pow(s_, 4) +
        a_coeffs[5] * std::pow(s_, 5);
}
double MinJerkCurve::evaluateFirstDerivative(const double &time) {
  s_ = clamp(time, to, tf);
  return a_coeffs[1] + 2.0 * a_coeffs[2] * s_ +
        3.0 * a_coeffs[3] * std::pow(s_, 2) +
        4.0 * a_coeffs[4] * std::pow(s_, 3) + 5.0 * a_coeffs[5] * std::pow(s_, 4);
}
double MinJerkCurve::evaluateSecondDerivative(const double &time) {
  s_ = clamp(time, to, tf);
  return 2.0 * a_coeffs[2] + 6.0 * a_coeffs[3] * s_ +
        12.0 * a_coeffs[4] * std::pow(s_, 2) +
        20.0 * a_coeffs[5] * std::pow(s_, 3);
}
