#include "quintic_hermite_curve.hpp"

#include <Eigen/Dense>
#include <iostream>

/// Quintic Hermite Interpolation
/// Implementation follows from:
/// https://studylib.net/doc/11701439/ma-323-geometric-modelling-course-notes--day-09-quintic-h...
  QuinticHermiteCurve::QuinticHermiteCurve() {
    p1 = 0;
    v1 = 0;
    a1 = 0;
    p2 = 0;
    v2 = 0;
    a2 = 0;
    t_dur = 0.5;
    s_ = 0;
  }

  QuinticHermiteCurve::QuinticHermiteCurve(const double &start_pos, const double &start_vel, const double &start_accel,
                                           const double &end_pos, const double &end_vel, const double &end_accel,
                                           const double &duration) {
    p1 = start_pos;
    v1 = start_vel;
    a1 = start_accel;
    p2 = end_pos;
    v2 = end_vel;
    a2 = end_accel;
    t_dur = duration;
    s_ = 0;

    if (t_dur < 1e-3) {
      std::cout << "given t_dur lower than minimum -> set to min: 0.001"
                << std::endl;
      t_dur = 1e-3;
    }
  }

QuinticHermiteCurve::~QuinticHermiteCurve() {}

double QuinticHermiteCurve::evaluate(const double &t_in) {
  s_ = this->clamp(t_in / t_dur);
  double H5_0 = 1. - 10. * std::pow(s_, 3)  + 15. * std::pow(s_, 4)  - 6. * std::pow(s_, 5);
  double H5_1 = s_  - 6. * std::pow(s_, 3) + 8. * std::pow(s_, 4) - 3 * std::pow(s_, 5);
  double H5_2 = 0.5 * s_ * s_ - 1.5 *std::pow(s_, 3) + 1.5 * std::pow(s_, 4) - 0.5 * std::pow(s_, 5);
  double H5_3 = 0.5 * std::pow(s_, 3) - std::pow(s_, 4) + 0.5 * std::pow(s_, 5);
  double H5_4 = -4. * std::pow(s_, 3) + 7. * std::pow(s_, 4) - 3. * std::pow(s_, 5);
  double H5_5 = 10. * std::pow(s_, 3) - 15. * std::pow(s_, 4) + 6. * std::pow(s_, 5);
  return p1 * H5_0 + v1 * H5_1 + a1 * H5_2 + a2 * H5_3 + v2 * H5_4 + p2 * H5_5;
}

double QuinticHermiteCurve::evaluateFirstDerivative(const double &t_in) {
  s_ = this->clamp(t_in / t_dur);
  double d_H5_0_dt = -30. * std::pow(s_, 2) + 60. * std::pow(s_, 3) - 30. * std::pow(s_, 4);
  double d_H5_1_dt = 1 - 18. * std::pow(s_, 2)  + 32. * std::pow(s_, 3) - 15. * std::pow(s_, 4);
  double d_H5_2_dt = s_ - 4.5 * std::pow(s_, 2) + 6. * std::pow(s_, 3) - 2.5 * std::pow(s_, 4);
  double d_H5_3_dt = 1.5 * std::pow(s_, 2) - 4. * std::pow(s_, 3) + 2.5 * std::pow(s_, 4);
  double d_H5_4_dt = -12. * std::pow(s_, 2) + 28. * std::pow(s_, 3) - 15. * std::pow(s_, 4);
  double d_H5_5_dt = 30. * std::pow(s_, 2) - 60. * std::pow(s_, 3) + 30. * std::pow(s_, 4);
  return (p1 * d_H5_0_dt + v1 * d_H5_1_dt + a1 * d_H5_2_dt + a2 * d_H5_3_dt + v2 * d_H5_4_dt
            + p2 * d_H5_5_dt) / t_dur;
}

double QuinticHermiteCurve::evaluateSecondDerivative(const double &t_in) {
  s_ = this->clamp(t_in / t_dur);
  double dd_H5_0_dt = -60 * s_ + 180. * std::pow(s_, 2) - 120. * std::pow(s_, 3);
  double dd_H5_1_dt = -36. * s_ + 96. * std::pow(s_, 2) - 60. * std::pow(s_, 3);
  double dd_H5_2_dt = 1 - 9. * s_ + 18. * std::pow(s_, 2) - 10. * std::pow(s_, 3);
  double dd_H5_3_dt = 3. * s_ - 12. * std::pow(s_, 2) + 10. * std::pow(s_, 3);
  double dd_H5_4_dt = -24. * s_ + 84. * std::pow(s_, 2) - 60. * std::pow(s_, 3);
  double dd_H5_5_dt = 60. * s_ - 180. * std::pow(s_, 2) + 120. * std::pow(s_, 3);
  return (p1 * dd_H5_0_dt + v1 * dd_H5_1_dt + a1 * dd_H5_2_dt + a2 * dd_H5_3_dt + v2 * dd_H5_4_dt
            + p2 * dd_H5_5_dt) / t_dur / t_dur;
}
