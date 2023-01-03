#pragma once

#include "hermite_curve.hpp"

class CubicHermiteCurve : public HermiteCurve {
public:
    CubicHermiteCurve();
    CubicHermiteCurve(const double &start_pos, const double &start_vel,
                      const double &end_pos, const double &end_vel,
                      const double &duration);
    ~CubicHermiteCurve();

    double evaluate(const double &t_in) override;
    double evaluateFirstDerivative(const double &t_in) override;
    double evaluateSecondDerivative(const double &t_in) override;

};
