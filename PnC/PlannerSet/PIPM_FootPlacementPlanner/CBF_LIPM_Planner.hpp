#pragma once

#include <Eigen/Dense>
#include <ExternalSource/myOptimizer/Goldfarb/QuadProg++.hh>
#include <Utils/IO/IOUtilities.hpp>
#include <vector>

class LinearInvertedPendulumModel;

class CBF_LIPM_Planner {
   public:
    CBF_LIPM_Planner();
    virtual ~CBF_LIPM_Planner();

    void PlannerInitialization(const YAML::Node& node);

    void SetControlBarrierFunction();
    void SetDynamics();
    Eigen::VectorXd getNextFootLocation(const Eigen::VectorXd& x_k0,
                                        const Eigen::VectorXd& p_k);

    void SetOmega(double w) {
        w_ = w;
        b_set_omega_ = true;
    }

   private:
    LinearInvertedPendulumModel* lipm_;

    double w_;
    bool b_set_omega_;
    Eigen::VectorXd x_step_length_limit_;
    Eigen::VectorXd y_step_length_limit_;

    int n_sol_;
    int n_ineq_;
    int n_eq_;
    // Sol : [px, py, epsilon]
    GolDIdnani::GVect<double> sol_;
    // Cost
    Eigen::MatrixXd G__;
    Eigen::VectorXd g0__;
    GolDIdnani::GMatr<double> G_;
    GolDIdnani::GVect<double> g0_;

    // Equality
    Eigen::MatrixXd CE__;
    Eigen::MatrixXd ce0__;
    GolDIdnani::GMatr<double> CE_;
    GolDIdnani::GVect<double> ce0_;

    // Inequality
    Eigen::MatrixXd CI__;
    Eigen::VectorXd ci0__;
    GolDIdnani::GMatr<double> CI_;
    GolDIdnani::GVect<double> ci0_;
};
