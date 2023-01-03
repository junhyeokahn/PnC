#pragma once

#include "hermite_curve.hpp"
#include <Eigen/Dense>

class MinJerkCurve : public HermiteCurve {
public:
    // Constructors
    MinJerkCurve();
    MinJerkCurve(const Eigen::Vector3d &init, const Eigen::Vector3d &end,
                 const double &time_start, const double &time_end);

    void SetParams(const Eigen::Vector3d &init, const Eigen::Vector3d &end,
                   const double &time_start, const double &time_end);

    double evaluate(const double &time) override;
    double evaluateFirstDerivative(const double &time) override;
    double evaluateSecondDerivative(const double &time) override;

    // Destructor
    ~MinJerkCurve();

private:
    Eigen::MatrixXd C_mat;     // Matrix of Coefficients
    Eigen::MatrixXd C_mat_inv; // Inverse of Matrix of Coefficients
    Eigen::VectorXd
            a_coeffs; // mininum jerk coeffs. a = [a0, a1, a2, a3, a4, a5, a6];
    Eigen::VectorXd bound_cond; // boundary conditions x_b = [ x(to), xdot(to),
    // xddot(to), x(tf), xdot(tf), xddot(tf)]

    Eigen::Vector3d init_cond; // initial pos, vel, acceleration
    Eigen::Vector3d end_cond;  // final pos, vel, acceleration
    double to;                 // Starting time
    double tf;                 // Ending time

    void Initialization();
};

