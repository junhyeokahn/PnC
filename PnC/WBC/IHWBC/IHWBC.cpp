#include <PnC/WBC/IHWBC/IHWBC.hpp>

IHWBC::IHWBC(const Eigen::MatrixXd &_sf, const Eigen::MatrixXd &_sa,
             const Eigen::MatrixXd &_sv) {
  util::PrettyConstructor(3, "IHWBC");

  n_q_dot_ = _sa.cols();
  n_active_ = _sa.rows();
  n_passive_ = _sv.rows();
  if (n_q_dot_ == n_active_ + n_passive_)
    b_floating_ = false;
  else
    b_floating_ = true;

  sf_ = _sf;
  snf_ =
      Eigen::MatrixXd::Zero(n_active_ + n_passive_, 6 + n_active_ + n_passive_);
  snf_.block(0, 6, n_active_ + n_passive_, n_active_ + n_passive_) =
      Eigen::MatrixXd::Identity(n_active_ + n_passive_, n_active_ + n_passive_);
  sv_ = _sv;
  sa_ = _sa;

  lambda_q_ddot = 0.;
  lambda_rf = 0.;
  w_rf = 0.;
}

IHWBC::~IHWBC() {}

void IHWBC::update_setting(const Eigen::MatrixXd &A,
                           const Eigen::MatrixXd &Ainv,
                           const Eigen::VectorXd &cori,
                           const Eigen::VectorXd &grav) {
  A_ = A;
  Ainv_ = Ainv;
  cori_ = cori;
  grav_ = grav;
}

void IHWBC::solve(
    const std::vector<Task *> &task_list,
    const std::vector<Contact *> &contact_list,
    const std::vector<InternalConstraint *> &internal_constraint_list,
    const Eigen::VectorXd &rf_des, Eigen::VectorXd &tau_cmd,
    Eigen::VectorXd &qddot_cmd, Eigen::VectorXd &rf_cmd) {

  assert(task_list.size() > 0);
  if (contact_list.size() > 0)
    b_contact_ = true;
  else
    b_contact_ = false;
  if (internal_constraint_list.size() > 0)
    b_internal_constraint_ = true;
  else
    b_internal_constraint_ = false;

  // ===========================================================================
  // Internal Constraint
  //   Set ni, jit_lmd_jidot_qdot, and sa_ni_trc_bar_tr
  // ===========================================================================
  Eigen::MatrixXd ni, sa_ni_trc_bar_tr;
  Eigen::VectorXd jit_lmd_jidot_qdot;

  Eigen::MatrixXd ji;
  Eigen::VectorXd jidot_qdot;
  if (b_internal_constraint_) {
    for (int i = 0; i < internal_constraint_list.size(); ++i) {
      if (i == 0) {
        ji = internal_constraint_list[i]->jacobian;
        jidot_qdot = internal_constraint_list[i]->jacobian_dot_q_dot;
      } else {
        ji = util::vStack(ji, internal_constraint_list[i]->jacobian);
        jidot_qdot = util::vStack(
            jidot_qdot, internal_constraint_list[i]->jacobian_dot_q_dot);
      }
    }
    Eigen::MatrixXd lmd_i;
    util::PseudoInverse(ji * Ainv_ * ji.transpose(), 0.001, lmd_i);
    Eigen::MatrixXd ji_bar = Ainv_ * ji.transpose() * lmd_i;
    ni = Eigen::MatrixXd::Identity(n_q_dot_, n_q_dot_) - ji_bar * ji;
    jit_lmd_jidot_qdot = ji.transpose() * lmd_i * jidot_qdot;
    Eigen::MatrixXd sa_ni_trc =
        (sa_ * ni).block(0, 6, n_active_, n_active_ + n_passive_);
    Eigen::MatrixXd lmd_sa_ni_trc;
    Eigen::MatrixXd Ainv_trc =
        Ainv_.block(6, 6, n_active_ + n_passive_, n_active_ + n_passive_);
    util::PseudoInverse(sa_ni_trc * Ainv_trc * sa_ni_trc.transpose(), 0.001,
                            lmd_sa_ni_trc);
    Eigen::MatrixXd sa_ni_trc_bar =
        Ainv_trc * sa_ni_trc.transpose() * lmd_sa_ni_trc;
    sa_ni_trc_bar_tr = sa_ni_trc_bar.transpose();
  } else {
    ni = Eigen::MatrixXd::Identity(n_q_dot_, n_q_dot_);
    jit_lmd_jidot_qdot = Eigen::VectorXd::Zero(n_q_dot_);
    sa_ni_trc_bar_tr = Eigen::MatrixXd::Identity(n_active_, n_active_);
  }
  // util::PrettyPrint(ni, std::cout, "ni");
  // util::PrettyPrint(jit_lmd_jidot_qdot, std::cout, "jit_lmd_jidot_qdot");
  // util::PrettyPrint(sa_ni_trc_bar_tr, std::cout, "sa_ni_trc_bar_tr");
  // exit(0);

  // ===========================================================================
  // Cost
  // ===========================================================================

  Eigen::MatrixXd cost_mat, cost_t_mat, cost_rf_mat;
  Eigen::VectorXd cost_vec, cost_t_vec, cost_rf_vec;

  cost_t_mat = Eigen::MatrixXd::Zero(n_q_dot_, n_q_dot_);
  cost_t_vec = Eigen::VectorXd::Zero(n_q_dot_);
  for (int i = 0; i < task_list.size(); ++i) {
    Eigen::MatrixXd jt = task_list[i]->jacobian;
    Eigen::VectorXd jt_dot_q_dot = task_list[i]->jacobian_dot_q_dot;
    Eigen::VectorXd x_ddot = task_list[i]->op_cmd;

    // std::cout << i << " th task" << std::endl;
    // task_list[i]->Debug();

    cost_t_mat += (w_hierarchy[i] * (jt.transpose() * jt));
    cost_t_vec += (w_hierarchy[i] * ((jt_dot_q_dot - x_ddot).transpose() * jt));
  }
  cost_t_mat += lambda_q_ddot * A_;

  Eigen::MatrixXd uf_mat, jc;
  Eigen::VectorXd uf_vec;
  if (b_contact_) {
    for (int i = 0; i < contact_list.size(); ++i) {
      if (i == 0) {
        uf_mat = contact_list[i]->cone_constraint_mat;
        uf_vec = contact_list[i]->cone_constraint_vec;
        jc = contact_list[i]->jacobian;
      } else {
        uf_mat =
            util::block_diag(uf_mat, contact_list[i]->cone_constraint_mat);
        uf_vec = util::vStack(uf_vec, contact_list[i]->cone_constraint_vec);
        jc = util::vStack(jc, contact_list[i]->jacobian);
      }
    }
    dim_cone_constraint_ = uf_mat.rows();
    dim_contacts_ = uf_mat.cols();

    cost_rf_mat = (lambda_rf + w_rf) *
                  Eigen::MatrixXd::Identity(dim_contacts_, dim_contacts_);
    cost_rf_vec = -w_rf * rf_des;

    cost_mat = util::block_diag(cost_t_mat, cost_rf_mat);
    cost_vec = util::vStack(cost_t_vec, cost_rf_vec);
  } else {
    dim_cone_constraint_ = 0;
    dim_contacts_ = 0;

    cost_mat = cost_t_mat;
    cost_vec = cost_t_vec;
  }

  // ===========================================================================
  // Equality Constraint
  // ===========================================================================

  Eigen::MatrixXd eq_floating_mat, eq_int_mat, eq_mat;
  Eigen::VectorXd eq_floating_vec, eq_int_vec, eq_vec;

  if (b_contact_) {
    eq_floating_mat = util::hStack(sf_ * A_, -sf_ * (jc * ni).transpose());
    if (b_internal_constraint_) {
      eq_int_mat =
          util::hStack(ji, Eigen::MatrixXd::Zero(ji.rows(), dim_contacts_));
      eq_int_vec = Eigen::VectorXd::Zero(ji.rows());
    }
  } else {
    eq_floating_mat = sf_ * A_;
    if (b_internal_constraint_) {
      eq_int_mat = ji;
      eq_int_vec = Eigen::VectorXd::Zero(ji.rows());
    }
  }
  eq_floating_vec = -sf_ * (ni.transpose() * (cori_ + grav_));

  if (b_internal_constraint_) {
    eq_mat = util::vStack(eq_floating_mat, eq_int_mat);
    eq_vec = util::vStack(eq_floating_vec, eq_int_vec);
  } else {
    eq_mat = eq_floating_mat;
    eq_vec = eq_floating_vec;
  }

  // ===========================================================================
  // Inequality Constraint
  // ===========================================================================

  Eigen::MatrixXd ineq_mat;
  Eigen::VectorXd ineq_vec;

  if (!b_trq_limit) {
    if (b_contact_) {
      ineq_mat = util::hStack(
          Eigen::MatrixXd::Zero(dim_cone_constraint_, n_q_dot_), -uf_mat);
      ineq_vec = -uf_vec;
    } else {
      assert(false); // Not Implemented (GoldIdnani doesn't solve this case)
    }
  } else {
    if (b_contact_) {
      Eigen::MatrixXd tmp1 = sa_ni_trc_bar_tr * snf_ * A_;
      Eigen::MatrixXd tmp2 = sa_ni_trc_bar_tr * snf_ * (jc * ni).transpose();
      ineq_mat = util::hStack(
          util::vStack(Eigen::MatrixXd::Zero(dim_cone_constraint_, n_q_dot_),
                          util::vStack(-tmp1, tmp1)),
          util::vStack(-uf_mat, util::vStack(tmp2, -tmp2)));
      Eigen::VectorXd tmp3 =
          sa_ni_trc_bar_tr * snf_ * ni.transpose() * (cori_ + grav_) +
          sa_ni_trc_bar_tr * snf_ * jit_lmd_jidot_qdot - trq_limit.col(0);
      Eigen::VectorXd tmp4 =
          -sa_ni_trc_bar_tr * snf_ * ni.transpose() * (cori_ + grav_) -
          sa_ni_trc_bar_tr * snf_ * jit_lmd_jidot_qdot + trq_limit.col(1);
      ineq_vec = util::vStack(-uf_vec, util::vStack(tmp3, tmp4));
    } else {
      Eigen::MatrixXd tmp1 = sa_ni_trc_bar_tr * snf_ * A_;
      ineq_mat = util::hStack(-tmp1, tmp1);
      Eigen::MatrixXd tmp2 =
          sa_ni_trc_bar_tr * snf_ * ni.transpose() * (cori_ + grav_) +
          sa_ni_trc_bar_tr * snf_ * jit_lmd_jidot_qdot - trq_limit.col(0);
      Eigen::MatrixXd tmp3 =
          -sa_ni_trc_bar_tr * snf_ * ni.transpose() * (cori_ + grav_) -
          sa_ni_trc_bar_tr * snf_ * jit_lmd_jidot_qdot + trq_limit.col(1);
      ineq_vec = util::vStack(tmp2, tmp3);
    }
  }

  // Solve Quadprog
  n_quadprog_ = cost_mat.cols();
  p_quadprog_ = ineq_mat.rows();
  m_quadprog_ = eq_mat.rows();

  x_.resize(n_quadprog_);
  G_.resize(n_quadprog_, n_quadprog_);
  g0_.resize(n_quadprog_);
  CE_.resize(n_quadprog_, m_quadprog_);
  ce0_.resize(m_quadprog_);
  CI_.resize(n_quadprog_, p_quadprog_);
  ci0_.resize(p_quadprog_);

  qp_dec_vars_ = Eigen::VectorXd::Zero(n_quadprog_);
  qddot_result_ = Eigen::VectorXd::Zero(n_q_dot_);
  fr_result_ = Eigen::VectorXd::Zero(dim_contacts_);

  /*
     min
        0.5 * x G x + g0 x
     s.t.
        CE^T x + ce0 = 0
        CI^T x + ci0 >= 0
    */
  setQuadProgCosts(cost_mat, cost_vec);
  setEqualityConstraints(-eq_mat, eq_vec);
  setInequalityConstraints(-ineq_mat, ineq_vec);
  solveQP();

  if (b_contact_) {
    tau_cmd = sa_ni_trc_bar_tr * snf_ *
              (A_ * qddot_result_ + ni.transpose() * (cori_ + grav_) -
               (jc * ni).transpose() * fr_result_);
  } else {
    tau_cmd =
        sa_ni_trc_bar_tr * snf_ * (A_ * qddot_result_ + ni * (cori_ + grav_));
  }
  qddot_cmd = sa_ * qddot_result_;
  rf_cmd = fr_result_;
  // util::PrettyPrint(tau_cmd, std::cout, "tau_cmd");
  // util::PrettyPrint(rf_cmd, std::cout, "rf_cmd");
  // util::PrettyPrint(qddot_result_, std::cout, "qddot_result");
  // exit(0);
}

void IHWBC::setQuadProgCosts(const Eigen::MatrixXd &P_cost,
                             const Eigen::VectorXd &v_cost) {
  // Set G
  for (int i = 0; i < n_quadprog_; i++) {
    for (int j = 0; j < n_quadprog_; j++) {
      G_[i][j] = P_cost(i, j);
    }
  }
  // Set g0
  for (int i = 0; i < n_quadprog_; i++) {
    g0_[i] = v_cost[i];
  }
}

void IHWBC::setEqualityConstraints(const Eigen::MatrixXd &Eq_mat,
                                   const Eigen::VectorXd &Eq_vec) {
  for (int i = 0; i < m_quadprog_; i++) {
    for (int j = 0; j < n_quadprog_; j++) {
      CE_[j][i] = Eq_mat(i, j);
    }
    ce0_[i] = Eq_vec[i];
  }
}

void IHWBC::setInequalityConstraints(const Eigen::MatrixXd &IEq_mat,
                                     const Eigen::VectorXd &IEq_vec) {
  for (int i = 0; i < p_quadprog_; ++i) {
    for (int j = 0; j < n_quadprog_; ++j) {
      CI_[j][i] = IEq_mat(i, j);
    }
    ci0_[i] = IEq_vec[i];
  }
}

void IHWBC::solveQP() {
  double qp_result = solve_quadprog(G_, g0_, CE_, ce0_, CI_, ci0_, x_);

  // Populate qd result from QP answer
  for (int i = 0; i < n_quadprog_; i++) {
    qp_dec_vars_[i] = x_[i];
  }
  // Store results
  qddot_result_ = qp_dec_vars_.head(n_q_dot_);
  fr_result_ = qp_dec_vars_.tail(dim_contacts_);

  // util::PrettyPrint(qp_dec_vars_, std::cout, "qp_dec_vars_");
  // util::PrettyPrint(qddot_result_, std::cout, "qddot_result_");
  // util::PrettyPrint(fr_result_, std::cout, "Fr_result_");
}
