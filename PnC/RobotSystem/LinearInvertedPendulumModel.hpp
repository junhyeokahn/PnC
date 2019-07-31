#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

class LinearInvertedPendulumModel {
   public:
    LinearInvertedPendulumModel(double w) : w_(w){};
    virtual ~LinearInvertedPendulumModel(){};

    /*
     * x_k(t) = Psi(t, t0; x_{k,0}, p_k) = Fpsi(t,t_0) x_{k,0} + Gpsi(t,t_0) p_k
     */
    Eigen::VectorXd Psi(double t, double t0, const Eigen::VectorXd& x_k0,
                        const Eigen::VectorXd& p_k) {
        Eigen::VectorXd ret = Eigen::VectorXd::Zero(4);
        ret = Fpsi(t, t0) * x_k0 + Gpsi(t, t0) * p_k;
        return ret;
    };

    Eigen::MatrixXd Fpsi(double t, double t0) {
        Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(4, 4);
        double delta_t = t - t0;

        ret(0, 0) = cosh(w_ * delta_t);
        ret(1, 1) = cosh(w_ * delta_t);
        ret(2, 2) = cosh(w_ * delta_t);
        ret(3, 3) = cosh(w_ * delta_t);

        ret(0, 2) = sinh(w_ * delta_t) / w_;
        ret(1, 3) = sinh(w_ * delta_t) / w_;

        ret(2, 0) = sinh(w_ * delta_t) * w_;
        ret(3, 1) = sinh(w_ * delta_t) * w_;

        return ret;
    };

    Eigen::MatrixXd Gpsi(double t, double t0) {
        Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(4, 2);
        double delta_t = t - t0;

        ret(0, 0) = 1 - cosh(w_ * delta_t);
        ret(1, 1) = 1 - cosh(w_ * delta_t);

        ret(2, 0) = -sinh(w_ * delta_t) * w_;
        ret(3, 1) = -sinh(w_ * delta_t) * w_;

        return ret;
    };

   private:
    double w_;
};
