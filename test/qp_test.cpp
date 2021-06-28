#include <Configuration.hpp>
#include <ExternalSource/myOptimizer/Goldfarb/QuadProg++.hh>
#include <Utils/IO/IOUtilities.hpp>
#include <iostream>

int main(int argc, char *argv[]) {
  Eigen::MatrixXd cost_mat, eq_mat, ineq_mat;
  Eigen::VectorXd cost_vec, eq_vec, ineq_vec;

  YAML::Node cfg = YAML::LoadFile(THIS_COM "ExperimentData/cpp.yaml");
  myUtils::readParameter(cfg, "cost_mat", cost_mat);
  myUtils::readParameter(cfg, "cost_vec", cost_vec);
  myUtils::readParameter(cfg, "eq_mat", eq_mat);
  myUtils::readParameter(cfg, "eq_vec", eq_vec);
  myUtils::readParameter(cfg, "ineq_mat", ineq_mat);
  myUtils::readParameter(cfg, "ineq_vec", ineq_vec);

  //

  // Eigen::MatrixXd M = Eigen::MatrixXd::Zero(3, 3);
  // M << 1., 2., 0., -8., 3., 2., 0., 1., 1.;
  // cost_mat = M.transpose() * M;
  // cost_vec = Eigen::Vector3d(3., 2., 3.).transpose() * M;
  // ineq_mat = Eigen::MatrixXd::Zero(3, 3);
  // ineq_mat << 1., 2., 1., 2., 0., 1., -1., 2., -1.;
  // ineq_vec = Eigen::VectorXd::Zero(3);
  // ineq_vec << 3., 2., -2.;
  // eq_mat = Eigen::MatrixXd::Zero(1, 3);
  // eq_vec = Eigen::VectorXd::Zero(1);
  // eq_mat << 1., 1., 1.;
  // eq_vec << 1.;

  //

  // myUtils::pretty_print(cost_mat, std::cout, "cost_mat");
  // myUtils::pretty_print(cost_vec, std::cout, "cost_vec");
  // myUtils::pretty_print(eq_mat, std::cout, "eq_mat");
  // myUtils::pretty_print(eq_vec, std::cout, "eq_vec");
  // myUtils::pretty_print(ineq_mat, std::cout, "ineq_mat");
  // myUtils::pretty_print(ineq_vec, std::cout, "ineq_vec");

  eq_mat = -eq_mat;
  ineq_mat = -ineq_mat;

  int n_quadprog_ = cost_mat.cols();
  int p_quadprog_ = ineq_mat.rows();
  int m_quadprog_ = eq_mat.rows();

  GolDIdnani::GVect<double> x_;
  GolDIdnani::GMatr<double> G_;
  GolDIdnani::GVect<double> g0_;
  GolDIdnani::GMatr<double> CE_;
  GolDIdnani::GVect<double> ce0_;
  GolDIdnani::GMatr<double> CI_;
  GolDIdnani::GVect<double> ci0_;

  x_.resize(n_quadprog_);
  G_.resize(n_quadprog_, n_quadprog_);
  g0_.resize(n_quadprog_);
  CE_.resize(n_quadprog_, m_quadprog_);
  ce0_.resize(m_quadprog_);
  CI_.resize(n_quadprog_, p_quadprog_);
  ci0_.resize(p_quadprog_);

  for (int i = 0; i < n_quadprog_; i++) {
    for (int j = 0; j < n_quadprog_; j++) {
      G_[i][j] = cost_mat(i, j);
    }
  }
  for (int i = 0; i < n_quadprog_; i++) {
    g0_[i] = cost_vec[i];
  }

  for (int i = 0; i < m_quadprog_; i++) {
    for (int j = 0; j < n_quadprog_; j++) {
      CE_[j][i] = eq_mat(i, j);
    }
    ce0_[i] = eq_vec[i];
  }

  for (int i = 0; i < p_quadprog_; ++i) {
    for (int j = 0; j < n_quadprog_; ++j) {
      CI_[j][i] = ineq_mat(i, j);
    }
    ci0_[i] = ineq_vec[i];
  }

  double qp_result = solve_quadprog(G_, g0_, CE_, ce0_, CI_, ci0_, x_);

  std::cout << "cost" << std::endl;
  std::cout << qp_result << std::endl;
  std::cout << "solution:" << std::endl;
  for (int i = 0; i < n_quadprog_; ++i) {
    std::cout << x_[i] << std::endl;
  }

  std::cout << "[Done]" << std::endl;
  return 0;
}
