#pragma once

#include <Utils/Math/MathUtilities.hpp>

// Implements the polynomial:
// x(t) = a0 + a_1*t + a_2*t^2 + a_3*t^3 
// Needs boundary conditions x(to), xdot(to), x(tf), xdot(tf)
// 		where to and tf are the initial and final times respectively. Also, tf > to.

class CubicFit_OneDimension{
    public:
        // Constructors
        CubicFit_OneDimension();
        CubicFit_OneDimension(const Eigen::Vector2d & init, const Eigen::Vector2d & end,
                const double & time_start, const double & time_end);

        void setParams(const Eigen::Vector2d & init, const Eigen::Vector2d & end,
                const double & time_start, const double & time_end);

        // Functions
        void test_func();

        void getPos(const double & time, double & pos);
        void getVel(const double & time, double & vel);
        void getAcc(const double & time, double & acc);

        void printParameters();

        // Destructor
        ~CubicFit_OneDimension();


    private:
        Eigen::MatrixXd C_mat; // Matrix of Coefficients
        Eigen::MatrixXd C_mat_inv;  // Inverse of Matrix of Coefficients
        Eigen::VectorXd a_coeffs;   // mininum jerk coeffs. a = [a0, a1, a2, a3];
        Eigen::VectorXd bound_cond; // boundary conditions x_b = [ x(to), xdot(to), x(tf), xdot(tf)]

        Eigen::Vector2d init_cond;  // initial pos, vel, acceleration
        Eigen::Vector2d end_cond;   // final pos, vel, acceleration
        double to; // Starting time
        double tf; // Ending time

        void Initialization();

        // Compute the coefficients
        void compute_coeffs();


};
