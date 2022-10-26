#pragma once

#include "pnc/whole_body_controllers/contact.hpp"
#include "pnc/whole_body_controllers/internal_constraint.hpp"
#include "pnc/whole_body_controllers/task.hpp"
#include "third_party//goldfarb/QuadProg++.hh"
#include "utils/util.hpp"

/// class IHWBC
class IHWBC {
public:
  /// \{ \name Constructor and Destructor
  IHWBC(const Eigen::MatrixXd &_sf, const Eigen::MatrixXd &_sa,
        const Eigen::MatrixXd &_sv);

  virtual ~IHWBC();
  /// \}

  /// Whether to enforce torque limit constraint.
  bool b_trq_limit = false;

  /// Torque limits of the actuating joints (size of n_a). The first and the
  /// second row represent min and max values, respectively.
  Eigen::MatrixXd trq_limit;

  /// Regularization weight for qddot.
  double lambda_q_ddot;

  /// Regularization weight for reaction force.
  double lambda_rf;
  Eigen::VectorXd lambda_rf_new;

  /// Weight for reaction force tracking.
  double w_rf;

  /// Update mass matrix, inverse of mass matrix, coriolis forces, gravity
  /// forces.
  void update_setting(const Eigen::MatrixXd &A, const Eigen::MatrixXd &Ainv,
                      const Eigen::VectorXd &cori, const Eigen::VectorXd &grav);

  /// Solve QP given tasks, contacts, internal constraints.
  void solve(const std::vector<Task *> &_task_list,
             const std::vector<Contact *> &_contact_list,
             const std::vector<InternalConstraint *> &_internal_constraint_list,
             const Eigen::VectorXd &_rf_des, Eigen::VectorXd &tau_cmd,
             Eigen::VectorXd &qddot_cmd, Eigen::VectorXd &rf_cmd,
             Eigen::VectorXd &f_int);

private:
  /// Number of qdot.
  int n_q_dot_;

  /// Number of floating dofs.
  int n_floating_;

  /// Number of actuating dofs.
  int n_active_;

  /// Number of passive dofs.
  int n_passive_;

  /// Number of contact constraints.
  int dim_cone_constraint_;

  /// Total contact dimension
  int dim_contacts_;

  /// Whether there is contact or not
  bool b_contact_;

  /// Whether there is internal constraint or not
  bool b_internal_constraint_;

  /// Whether there is internal constraint or not
  bool b_floating_;

  /// Selection matrix for floating dofs.
  Eigen::MatrixXd sf_;

  /// Selection matrix for non floating dofs.
  Eigen::MatrixXd snf_;

  /// Selection matrix for actuating dofs.
  Eigen::MatrixXd sa_;

  /// Selection matrix for passive dofs.
  Eigen::MatrixXd sv_;

  /// Mass matrix.
  Eigen::MatrixXd A_;

  /// Inverse of mass matrix.
  Eigen::MatrixXd Ainv_;

  /// Coriolis forces.
  Eigen::VectorXd cori_;

  /// Gravity forces.
  Eigen::VectorXd grav_;

  /// Number of decision variables.
  int n_quadprog_ = 1;

  /// Number of inequality constraints.
  int p_quadprog_ = 0;

  /// Number of equality constraints.
  int m_quadprog_ = 0;

  /// QP variables.
  GolDIdnani::GVect<double> x_;

  /// QP cost.
  GolDIdnani::GMatr<double> G_;
  GolDIdnani::GVect<double> g0_;

  /// QP equality cosntraint.
  GolDIdnani::GMatr<double> CE_;
  GolDIdnani::GVect<double> ce0_;

  /// QP inequality constraint.
  GolDIdnani::GMatr<double> CI_;
  GolDIdnani::GVect<double> ci0_;

  /// QP Result Containers.
  Eigen::VectorXd qp_dec_vars_;
  Eigen::VectorXd qddot_result_;
  Eigen::VectorXd fr_result_;

  /// Set QP cost.
  void setQuadProgCosts(const Eigen::MatrixXd &mat, const Eigen::VectorXd &vec);

  /// Set equality constraints.
  void setEqualityConstraints(const Eigen::MatrixXd &mat,
                              const Eigen::VectorXd &vec);

  /// Set null equality constraints.
  void setNullEqualityConstraints();

  /// Set inequality constraints.
  void setInEqualityConstraints(const Eigen::MatrixXd &mat,
                                const Eigen::VectorXd &vec);

  /// Set null inequality constraints.
  void setNullInEqualityConstraints();

  /// Solve QP.
  void solveQP();
};
