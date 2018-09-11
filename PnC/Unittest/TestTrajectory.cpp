#include "gtest/gtest.h"
#include <drake/manipulation/util/trajectory_utils.h>
#include "drake/common/trajectories/piecewise_polynomial.h"
#include <iostream>
#include <Eigen/Dense>

TEST(TestTrajectory, Cubic1) {
    std::vector<double> time(2);
    std::vector< Eigen::MatrixXd > knots(2, Eigen::MatrixXd::Zero(2, 1));
    std::vector< Eigen::MatrixXd > knots_dot(2, Eigen::MatrixXd::Zero(2,1));
    time[0] = 0.; time[1] = 2.;
    knots[0] << 0., 0.; knots[1] << 8., 16.;
    knots_dot[0] << 0., 0.; knots_dot[1] << 12., 24.;
    drake::trajectories::PiecewisePolynomial<double> ppos = drake::trajectories::PiecewisePolynomial<double>::Cubic(time, knots, knots_dot);

    drake::trajectories::PiecewisePolynomial<double> pvel = ppos.derivative();
    drake::trajectories::PiecewisePolynomial<double> pacc = pvel.derivative();
    std::cout << ppos.value(0) << std::endl;
    std::cout << ppos.value(1) << std::endl;
    std::cout << ppos.value(2) << std::endl;
    std::cout << pvel.value(0) << std::endl;
    std::cout << pvel.value(1) << std::endl;
    std::cout << pvel.value(2) << std::endl;
    std::cout << pacc.value(0) << std::endl;
    std::cout << pacc.value(1) << std::endl;
    std::cout << pacc.value(2) << std::endl;
};

TEST(TestTrajectory, Cubic2) {
    std::vector<double> time(2);
    std::vector< Eigen::MatrixXd > knots(2);
    std::vector< Eigen::MatrixXd > knots_dot(2);
    time[0] = 0.; time[1] = 2.;
    std::vector<Eigen::VectorXd> vknots(2, Eigen::VectorXd::Zero(2));
    std::vector<Eigen::VectorXd> vknots_dot(2, Eigen::VectorXd::Zero(2));
    vknots[0] << 0., 0.; vknots[1] << 8., 16.;
    vknots_dot[0] << 0., 0.; vknots_dot[1] << 12., 24.;
    knots[0] = vknots[0];
    knots[1] = vknots[1];
    knots_dot[0] = vknots_dot[0];
    knots_dot[1] = vknots_dot[1];
    drake::trajectories::PiecewisePolynomial<double> ppos =
        drake::trajectories::PiecewisePolynomial<double>::Cubic(time, knots,
                knots_dot);

    drake::trajectories::PiecewisePolynomial<double> pvel = ppos.derivative();
    drake::trajectories::PiecewisePolynomial<double> pacc = pvel.derivative();
    std::cout << ppos.value(0) << std::endl;
    std::cout << ppos.value(1) << std::endl;
    std::cout << ppos.value(2) << std::endl;
    std::cout << pvel.value(0) << std::endl;
    std::cout << pvel.value(1) << std::endl;
    std::cout << pvel.value(2) << std::endl;
    std::cout << pacc.value(0) << std::endl;
    std::cout << pacc.value(1) << std::endl;
    std::cout << pacc.value(2) << std::endl;
};
