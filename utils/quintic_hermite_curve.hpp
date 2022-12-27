#pragma once

#include "hermite_curve.hpp"

class QuinticHermiteCurve : public HermiteCurve {
  public:
    QuinticHermiteCurve();
    QuinticHermiteCurve(const double &start_pos, const double &start_vel,
                        const double &start_accel, const double &end_pos,
                        const double &end_vel, const double &end_accel,
                        const double &duration);
    ~QuinticHermiteCurve();
    double evaluate(const double &t_in) override;
    double evaluateFirstDerivative(const double &t_in) override;
    double evaluateSecondDerivative(const double &t_in) override;

};
