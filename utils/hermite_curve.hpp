#pragma once

#include<iostream>

class HermiteCurve {
public:
  HermiteCurve();
  virtual ~HermiteCurve() = 0;
  virtual double evaluate(const double &t_in) = 0;
  virtual double evaluateFirstDerivative(const double &t_in) = 0;
  virtual double evaluateSecondDerivative(const double &t_in) = 0;

  inline double clamp(const double &s_in, double lo=0.0, double hi=1.0) {
    if (s_in < lo) {
      return lo;
    } else if (s_in > hi) {
      return hi;
    } else {
      return s_in;
    }
  }

protected:
  double p1;
  double v1;
  double a1;
  double p2;
  double v2;
  double a2;
  double t_dur;

  double s_;
};
