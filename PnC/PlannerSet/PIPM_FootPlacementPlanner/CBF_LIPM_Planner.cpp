#include <PnC/PlannerSet/PIPM_FootPlacementPlanner/CBF_LIPM_Planner.hpp>
#include <PnC/RobotSystem/LinearInvertedPendulumModel.hpp>
#include <cassert>

CBF_LIPM_Planner::CBF_LIPM_Planner()
    : b_set_omega_(false), n_sol_(3), n_ineq_(3), n_eq_(1) {
    G_.resize(n_sol_, n_sol_);
    g0_.resize(n_sol_);
    CI_.resize(n_sol_, n_ineq_);
    ci0_.resize(n_ineq_);
    CE_.resize(n_sol_, n_eq_);
    ce0_.resize(n_eq_);
}

CBF_LIPM_Planner::~CBF_LIPM_Planner() { delete lipm_; }

void CBF_LIPM_Planner::PlannerInitialization(const YAML::Node& node) {
    try {
        myUtils::readParameter(node, "x_step_length_limit",
                               x_step_length_limit_);
        myUtils::readParameter(node, "y_step_length_limit",
                               y_step_length_limit_);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }

    assert(b_set_omega_);
    lipm_ = new LinearInvertedPendulumModel(w_);
}

void CBF_LIPM_Planner::SetControlBarrierFunction() {}

void CBF_LIPM_Planner::SetDynamics() {}

Eigen::VectorXd CBF_LIPM_Planner::getNextFootLocation(
    const Eigen::VectorXd& x_k0, const Eigen::VectorXd& p_k) {
    Eigen::VectorXd ret = Eigen::VectorXd::Zero(2);

    // =========================================================================
    // Cost
    // =========================================================================
    G__ = Eigen::MatrixXd::Zero(n_sol_, n_sol_);
    G__(0, 0) = 1.0;
    G__(1, 1) = 1.0;
    G__(2, 2) = 1000000.0;
    g0__ = Eigen::VectorXd::Zero(n_sol_);
    for (int row_idx = 0; row_idx < n_sol_; ++row_idx) {
        for (int col_idx = 0; col_idx < n_sol_; ++col_idx) {
            G_[row_idx][col_idx] = G__(row_idx, col_idx);
        }
    }
    for (int idx = 0; idx < n_sol_; ++idx) g0_[idx] = g0__(idx);

    // =========================================================================
    // Inequality Constraint
    // =========================================================================
    CI__ = Eigen::MatrixXd::Zero(n_sol_, n_ineq_);
    ci0__ = Eigen::VectorXd::Zero(n_ineq_);
    // TODO

    // =========================================================================
    // Equality Constraint
    // =========================================================================
    CE__ = Eigen::MatrixXd::Zero(n_sol_, n_eq_);
    ce0__ = Eigen::VectorXd::Constant(n_eq_, 1.0);
    for (int row_idx = 0; row_idx < n_sol_; ++row_idx) {
        for (int col_idx = 0; col_idx < n_eq_; ++col_idx) {
            CE_[row_idx][col_idx] = CE__(row_idx, col_idx);
        }
    }
    for (int idx = 0; idx < n_eq_; ++idx) ce0_[idx] = ce0__(idx);

    // =========================================================================
    // Solve
    // =========================================================================
    double f = solve_quadprog(G_, g0_, CE_, ce0_, CI_, ci0_, sol_);
    ret << sol_[0], sol_[1];

    return ret;
}
