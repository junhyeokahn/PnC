#pragma once

#include <Utils/Math/MathUtilities.hpp>

// Implements the polynomial:
// x(t) = a0 + a_1*t 
// Needs boundary conditions x(to), x(tf)
// 		where to and tf are the initial and final times respectively. Also, tf > to.

class LinearFit_OneDimension{
    public:
        // Constructors
        LinearFit_OneDimension();
        LinearFit_OneDimension(const double init, const double end,
                const double time_start, const double time_end);

        void setParams(double init, double end,
                const double time_start, const double time_end);

        // Functions
        void test_func();

        void getPos(const double time, double & pos);
        void getVel(const double time, double & vel);
        void getAcc(const double time, double & acc);

        void printParameters();

        // Destructor
        ~LinearFit_OneDimension();


    private:
        Eigen::MatrixXd C_mat; // Matrix of Coefficients
        Eigen::MatrixXd C_mat_inv;  // Inverse of Matrix of Coefficients
        Eigen::VectorXd a_coeffs;   // linear coeffs [a0, a1]
        Eigen::VectorXd bound_cond; // boundary conditions x_b = [ x(to), x(tf) ]

        double init_cond;  // initial pos
        double end_cond;   // final pos
        double to; // Starting time
        double tf; // Ending time

        void Initialization();

        // Compute the coefficients
        void compute_coeffs();


};
