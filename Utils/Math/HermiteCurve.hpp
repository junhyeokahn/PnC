#ifndef ALM_HERMITE_CURVE_H
#define ALM_HERMITE_CURVE_H

#include <algorithm>
#include <cmath>
#include <iostream>

// returns a hermite interpolation (cubic) of the boundary conditions for a
// given s \in [0,1].

class HermiteCurve {
public:
  HermiteCurve();
  HermiteCurve(const double &start_pos, const double &start_vel,
               const double &end_pos, const double &end_vel);
  ~HermiteCurve();
  double evaluate(const double &s_in);
  double evaluateFirstDerivative(const double &s_in);
  double evaluateSecondDerivative(const double &s_in);

private:
  double p1;
  double v1;
  double p2;
  double v2;

  double s_;

  // by default clamps within 0 and 1.
  double clamp(const double &s_in, double lo = 0.0, double hi = 1.0);
};

#endif
