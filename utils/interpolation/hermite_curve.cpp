#include "hermite_curve.hpp"

HermiteCurve::HermiteCurve() {
  p1 = 0.;
  v1 = 0.;
  a1 = 0.;
  p2 = 0.;
  v2 = 0.;
  a2 = 0.;
  t_dur = 0.;
  s_= 0.;
}

HermiteCurve::~HermiteCurve() = default;