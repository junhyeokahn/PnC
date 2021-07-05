#pragma once

#include "utils/util.hpp"
#include <ExternalSource/myOptimizer/Goldfarb/QuadProg++.hh>
#include <PnC/WBC/Contact.hpp>
#include <PnC/WBC/InternalConstraint.hpp>
#include <PnC/WBC/Task.hpp>

// Implicit Hierarchical Whole Body Controller
class IHWBC {
public:
  IHWBC(const Eigen::MatrixXd &_sf, const Eigen::MatrixXd &_sa,
        const Eigen::MatrixXd &_sv);
  virtual ~IHWBC();

  bool b_trq_limit = false;
  Eigen::MatrixXd trq_limit;

  double lambda_q_ddot;
  double lambda_rf;
  double w_rf;
  Eigen::VectorXd w_hierarchy;

  void update_setting(const Eigen::MatrixXd &A, const Eigen::MatrixXd &Ainv,
                      const Eigen::VectorXd &cori, const Eigen::VectorXd &grav);

  void solve(const std::vector<Task *> &_task_list,
             const std::vector<Contact *> &_contact_list,
             const std::vector<InternalConstraint *> &_internal_constraint_list,
             const Eigen::VectorXd &_rf_des, Eigen::VectorXd &tau_cmd,
             Eigen::VectorXd &qddot_cmd, Eigen::VectorXd &rf_cmd);

private:
  int n_q_dot_;
  int n_active_;
  int n_passive_;

  int dim_cone_constraint_;
  int dim_contacts_;

  bool b_contact_;
  bool b_internal_constraint_;
  bool b_floating_;

  Eigen::MatrixXd sf_;
  Eigen::MatrixXd snf_;
  Eigen::MatrixXd sa_;
  Eigen::MatrixXd sv_;

  Eigen::MatrixXd A_;
  Eigen::MatrixXd Ainv_;
  Eigen::VectorXd cori_;
  Eigen::VectorXd grav_;

  // Quadprog sizes
  int n_quadprog_ = 1; // Number of Decision Variables
  int p_quadprog_ = 0; // Number of Inequality Constraints
  int m_quadprog_ = 0; // Number of Equality Constraints

  // Quadprog Variables
  GolDIdnani::GVect<double> x_;
  // Cost
  GolDIdnani::GMatr<double> G_;
  GolDIdnani::GVect<double> g0_;

  // Equality
  GolDIdnani::GMatr<double> CE_;
  GolDIdnani::GVect<double> ce0_;

  // Inequality
  GolDIdnani::GMatr<double> CI_;
  GolDIdnani::GVect<double> ci0_;

  // Quadprog Result Containers
  Eigen::VectorXd qp_dec_vars_;
  Eigen::VectorXd qddot_result_;
  Eigen::VectorXd fr_result_;

  void setQuadProgCosts(const Eigen::MatrixXd &mat, const Eigen::VectorXd &vec);
  void setEqualityConstraints(const Eigen::MatrixXd &mat,
                              const Eigen::VectorXd &vec);
  void setInequalityConstraints(const Eigen::MatrixXd &mat,
                                const Eigen::VectorXd &vec);
  void solveQP();
};
