#include "cubic_hermite_curve.hpp"
#include <Eigen/Dense>
#include <iostream>

CubicHermiteCurve::CubicHermiteCurve() {
  p1 = 0;
  v1 = 0;
  p2 = 0;
  v2 = 0;
  t_dur = 0.5;
  s_ = 0;
}

CubicHermiteCurve::CubicHermiteCurve(const double &start_pos, const double &start_vel,
                                     const double &end_pos, const double &end_vel,
                                     const double &duration) {
  p1 = start_pos;
  v1 = start_vel;
  p2 = end_pos;
  v2 = end_vel;
  t_dur = duration;
  s_ = 0;

  if (t_dur < 1e-3) {
    std::cout << "given t_dur lower than minimum -> set to min: 0.001"
              << std::endl;
    t_dur = 1e-3;
  }
}


CubicHermiteCurve::~CubicHermiteCurve() {}

// Cubic Hermite HermiteCurve:
// From https://en.wikipedia.org/wiki/Cubic_Hermite_spline#Unit_interval_(0,_1)
// p(s) = (2s^3 - 3s^2 + 1)*p1 + (-2*s^3 + 3*s^2)*p2 + (s^3 - 2s^2 + s)*v1 +
// (s^3 - s^2)*v2 where 0 <= s <= 1.
double CubicHermiteCurve::evaluate(const double &t_in) {
  s_ = this->clamp(t_in / t_dur);
  return p1 * (2 * std::pow(s_, 3) - 3 * std::pow(s_, 2) + 1) +
       p2 * (-2 * std::pow(s_, 3) + 3 * std::pow(s_, 2)) +
       v1 * t_dur * (std::pow(s_, 3) - 2 * std::pow(s_, 2) + s_) +
       v2 * t_dur * (std::pow(s_, 3) - std::pow(s_, 2));
}

double CubicHermiteCurve::evaluateFirstDerivative(const double &t_in) {
  s_ = this->clamp(t_in / t_dur);
  return (p1 * (6 * std::pow(s_, 2) - 6 * s_) +
          p2 * (-6 * std::pow(s_, 2) + 6 * s_) +
          v1 * t_dur * (3 * std::pow(s_, 2) - 4 * s_ + 1) +
          v2 * t_dur * (3 * std::pow(s_, 2) - 2 * s_)) /
         t_dur;
}

double CubicHermiteCurve::evaluateSecondDerivative(const double &t_in) {
  s_ = this->clamp(t_in / t_dur);
  return (p1 * (12 * s_ - 6) + p2 * (-12 * s_ + 6) + v1 * t_dur * (6 * s_ - 4) +
          v2 * t_dur * (6 * s_ - 2)) /
         t_dur / t_dur;
}

