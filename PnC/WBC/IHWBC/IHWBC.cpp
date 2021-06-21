#include <PnC/WBC/IHWBC/IHWBC.hpp>

IHWBC::IHWBC(const Eigen::MatrixXd &_sf, const Eigen::MatrixXd &_sa,
             const Eigen::MatrixXd &_sv) {
  myUtils::pretty_constructor(3, "IHWBC");

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
    const std::vector<ContactSpec *> &contact_list,
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
  if (b_internal_constraint_) {
    Eigen::MatrixXd ji;
    Eigen::VectorXd jidot_qdot;
    for (int i = 0; i < internal_constraint_list.size(); ++i) {
      if (i == 0) {
        ji = internal_constraint_list[i].jacobian;
        jidot_qdot = internal_constraint_list[i].jacobian_dot_q_dot;
      } else {
        ji = myUtils::hStack(ji, internal_constraint_list[i].jacobian);
        jidot_qdot = myUtils::vStack(
            jidot_qdot, internal_constraint_list[i].jacobian_dot_q_dot);
      }
    }
    Eigen::MatrixXd lmd_i;
    myUtils::pseudo_inverse::(ji * Ainv_ * ji.transpose(), 0.001, lmd_i);
    Eigen::MatrixXd ji_bar = Ainv_ * ji.transpose() * lmd_i;
    ni = Eigen::MatrixXd::Identity(n_q_dot_, n_q_dot_) - ji_bar * ji;
    jit_lmd_jidot_qdot = ji.transpose() * lmd_i * jidot_qdot;
    Eigen::MatrixXd sa_ni_trc =
        (sa_ * ni).block(0, 6, n_active_, n_active_ + n_passive_);
    Eigen::MatrixXd lmd_sa_ni_trc;
    Eigen::MatrixXd Ainv_trc =
        Ainv_.block(6, 6, n_active_ + n_passive_, n_active_ + n_passive_);
    myUtils::pseudo_inverse(sa_ni_trc * A_inv_trc * sa_ni_trc.transpose(),
                            0.001, lmd_sa_ni_trc);
    Eigen::MatrixXd sa_ni_trc_bar =
        Ainv_trc * sa_ni_trc.transpose() * lmd_sa_ni_trc;
    sa_ni_trc_bar_tr = sa_ni_trc_bar.transpose();
  } else {
    ni = Eigen::MatrixXd::Identity(n_q_dot_, n_q_dot_);
    jit_lmd_jidot_qdot = Eigen::VectorXd::Zero(n_q_dot_);
    sa_ni_trc_bar_tr = Eigen::MatrixXd::Identity(n_active_, n_active_);
  }

  // ===========================================================================
  // Cost matrix
  // ===========================================================================
  Eigen::MatrixXd cost_t_mat = Eigen::MatrixXd::Zero(n_q_dot_, n_q_dot_);
  Eigen::VectorXd cost_t_vec = Eigen::VectorXd::Zero(n_q_dot);

  for (int i = 0; i < task_list.size(); ++i) {
    Eigen::MatrixXd jt = task_list[i]->jacobian;
    Eigen::VectorXd jt_dot_q_dot = task_list[i]->jacobian_dot_q_dot;
    Eigen::VectorXd x_ddot = task_list[i]->op_cmd;

    cost_t_mat += (w_hierarchy[i] * (jt.transpose() * jt));
    cost_t_vec += (w_hierarchy[i] * ((jt_dot_q_dot - x_ddot).transpose() * jt));
  }
  cost_t_mat += lambda_q_ddot * A_;

  Eigen::MatrixXd uf_mat;
  Eigen::VectorXd uf_vec;
  if (b_contact_) {
    for (int i = 0; i < contact_list.size(); ++i) {
      if (i == 0) {
        uf_mat = contact_list[i]->cone_constraint_mat;
        uf_vec = contact_list[i]->
      } else {
      }
    }
  } else {
  }

  // Prepare contact dimensions
  dim_contacts_ = 0;
  dim_contact_constraints_ = 0;
  if (contact_list.size() > 0) {
    // Construct Contact Jacobians
    buildContactStacks(contact_list, w_rf_contacts);

    if (!target_wrench_minimization) {
      // w_f*||Fd - Fr||^2_2 + lambda_Fr*||Fr||
      // Target Force Minimization (term by term)

      // Force Contact Costs
      Pf = (w_contact_weight + lambda_Fr) *
           Eigen::MatrixXd::Identity(dim_contacts_, dim_contacts_);
      vf = -w_contact_weight * Fd.transpose() *
           Eigen::MatrixXd::Identity(dim_contacts_, dim_contacts_);
    } else {
      // Target Wrench Minimization
      // TODO: Modify to perform adjoint mapping to CoM frame.
      // w_f * (sum_i=1^n ||w_i*Fd - Sf J_c_i^T Fr_i ||^2_2) +
      // lambda_Fr*||Fr||^2_2

      Pf = Eigen::MatrixXd::Zero(dim_contacts_, dim_contacts_);
      vf = Eigen::VectorXd::Zero(dim_contacts_);
      Eigen::MatrixXd Jr(6, dim_contacts_);
      int dim_rf_prev = 0;
      int dim_rf = 0;
      for (int i = 0; i < contact_list.size(); i++) {
        dim_rf = contact_list[i]->getDim();
        Jr.setZero();
        Jr.block(0, dim_rf_prev, 6, contact_list[i]->getDim()) =
            Sf_ * Jc_list_[i].transpose();
        Pf += w_contact_weight * (Jr.transpose() * Jr);
        vf += w_contact_weight * w_rf_contacts[i] * (-Fd.transpose() * Jr);
        dim_rf_prev += dim_rf;
      }
      Pf += lambda_Fr * Eigen::MatrixXd::Identity(dim_contacts_, dim_contacts_);
    }
  }

  // Set total cost matrix and vector
  Eigen::MatrixXd Ptot(num_qdot_ + dim_contacts_, num_qdot_ + dim_contacts_);
  Eigen::VectorXd vtot(num_qdot_ + dim_contacts_);
  Ptot.setZero();
  vtot.setZero();

  // Assign Block Cost Matrices
  Ptot.block(0, 0, num_qdot_, num_qdot_) = Pt;
  vtot.head(num_qdot_) = vt;
  if (dim_contacts_ > 0) {
    Ptot.block(num_qdot_, num_qdot_, dim_contacts_, dim_contacts_) = Pf;
    vtot.tail(dim_contacts_) = vf;
  }

  dim_dec_vars_ = num_qdot_ + dim_contacts_;

  // Create the Dynamic Equality Constraint
  // CE^T = Sf(A - Jc^T )
  // ce0 = Sf(b + g)
  Eigen::MatrixXd dyn_CE(6, dim_dec_vars_);
  Eigen::VectorXd dyn_ce0(6);
  // Floating Base Dynamics
  dyn_CE.block(0, 0, 6, num_qdot_) = Sf_ * A_;
  dyn_CE.block(0, num_qdot_, 6, dim_contacts_) = -Sf_ * Jc_.transpose();
  dyn_ce0 = Sf_ * (cori_ + grav_);

  // Create the Dynamic Inequality Constraint
  dim_inequality_constraints_ = dim_contact_constraints_;
  // Check if torque limits are enabled
  if (b_torque_limits_) {
    dim_inequality_constraints_ += 2 * num_act_joint_;
  }

  Eigen::MatrixXd dyn_CI(dim_inequality_constraints_, dim_dec_vars_);
  Eigen::VectorXd dyn_ci0(dim_inequality_constraints_);
  dyn_CI.setZero();
  dyn_ci0.setZero();

  // Contact Constraints
  // U*Fr + *ci0*Fr >= 0
  if (dim_contact_constraints_ > 0) {
    dyn_CI.block(0, num_qdot_, dim_contact_constraints_, dim_contacts_) = Uf_;
    dyn_ci0.segment(0, dim_contact_constraints_) = -uf_ieq_vec_;
  }

  int row_offset = dim_contact_constraints_;
  if (b_torque_limits_) {
    // Torque limits
    // Sa_(Aqddot - Jc_.transpose*Fr + cori_ + grav_ ) - tau_min >= 0
    dyn_CI.block(row_offset, 0, num_act_joint_, num_qdot_) = Sa_ * A_;
    dyn_CI.block(row_offset, num_qdot_, num_act_joint_, dim_contacts_) =
        -Sa_ * Jc_.transpose();
    dyn_ci0.segment(row_offset, num_act_joint_) =
        Sa_ * (cori_ + grav_) - tau_min_;
    // -Sa_(Aqddot - Jc_.transpose*Fr + cori_ + grav_ ) + tau_max >= 0
    dyn_CI.block(row_offset + num_act_joint_, 0, num_act_joint_, num_qdot_) =
        -Sa_ * A_;
    dyn_CI.block(row_offset + num_act_joint_, num_qdot_, num_act_joint_,
                 dim_contacts_) = Sa_ * Jc_.transpose();
    dyn_ci0.segment(row_offset + num_act_joint_, num_act_joint_) =
        -Sa_ * (cori_ + grav_) + tau_max_;
  }

  // myUtils::pretty_print(Uf_, std::cout, "Uf_");
  // myUtils::pretty_print(uf_ieq_vec_, std::cout, "uf_ieq_vec_");
  // myUtils::pretty_print(dyn_ci0, std::cout, "dyn_ci0");

  // To Do: Joint Limit Constraints
  // Naive formulation:
  // q + qdot*dt + qddot*dt*dt/2 >= q_min
  //  => qddot*dt*dt/2 + (qdot*dt + q - q_min) >= 0
  // q + qdot*dt + qddot*dt*dt/2 <= q_max
  //  => -(qddot*dt*dt/2 + qdot*dt + q) + q_max >= 0
  //  => -qddot*dt*dt/2  (-qdot*dt - q + q_max) >= 0
  // Better Formulation which considers velocity sign changes and viability
  // qddot_min <= qddot <= qddot_max (from Del Prete: Del Prete, Andrea. "Joint
  // position and velocity bounds in discrete-time acceleration/torque control
  // of robot manipulators." IEEE Robotics and Automation Letters 3.1 (2017):
  // 281-288.
  // => qddot - qddot_min >= 0
  // => -qddot + qddot_max >= 0

  // Solve Quadprog
  prepareQPSizes();                          // Prepare QP size
  setQuadProgCosts(Ptot, vtot);              // Set Quadprog Costs
  setEqualityConstraints(dyn_CE, dyn_ce0);   // Create Equality Constraints
  setInequalityConstraints(dyn_CI, dyn_ci0); // Create Inequality Constraints
  solveQP();

  if (contact_list.size() > 0) {
    tau_cmd = Sa_ * (A_ * qddot_result_ + cori_ + grav_ -
                     Jc_.transpose() * Fr_result_);
  } else {
    tau_cmd = Sa_ * (A_ * qddot_result_ + cori_ + grav_);
  }

  qddot_cmd = Sa_ * qddot_result_;

  // for(int i = 0; i < task_list.size(); i++){
  //     task = task_list[i];
  //     task->getTaskJacobian(Jt);
  //     task->getTaskJacobianDotQdot(JtDotQdot);
  //     task->getCommand(xddot);
  //     std::cout << "task " << i << " error : " << (xddot - (Jt*qddot_result_)
  //     -  JtDotQdot).transpose() << std::endl;
  //     std::cout << "task " << i << " error norm : " << (xddot -
  //     (Jt*qddot_result_) -  JtDotQdot).norm() << std::endl;
  //     std::cout << "    " << " xddot : " << (xddot).transpose() << std::endl;
  //     std::cout << "    " << " Jtqddot : " << (Jt*qddot_result_).transpose()
  //     << std::endl;
  //     std::cout << "    " << " JtDotQdot : " << JtDotQdot.transpose() <<
  //     std::endl;
  // }

  // Eigen::VectorXd tau_res_test = A_*qddot_result_ + cori_ + grav_ -
  // Jc_.transpose()*Fr_result_;
  // myUtils::pretty_print(tau_res_test, std::cout, "tau_res_test");
  // Eigen::VectorXd qddot_res_test = Ainv_*(tau_res_test - cori_ - grav_ +
  // Jc_.transpose()*Fr_result_);
  // myUtils::pretty_print(qddot_res_test, std::cout, "qddot_res_test");
  // myUtils::pretty_print(qddot_result_, std::cout, "qddot_result_");

  // TEST
  // for (int i = 0; i < task_list.size(); ++i) {
  // task_list[i]->getCommand(xddot);
  // myUtils::saveVector(xddot, "debug_wbc_task_" + std::to_string(i));
  //}
  // myUtils::saveVector(Fr_result_, "debug_wbc_rf");
  // myUtils::saveVector(qddot_result_, "debug_wbc_qddot");
  // myUtils::saveVector(tau_cmd, "debug_wbc_tau_cmd");
  // TEST
}

// Creates a stack of contact jacobians that are weighted by w_rf_contacts
// Also create the contact constraints
void IHWBC::buildContactStacks(const std::vector<ContactSpec *> &contact_list,
                               const Eigen::VectorXd &w_rf_contacts_in) {
  // Temporary containers
  // Contact Constraints
  Eigen::MatrixXd Uf;
  Eigen::VectorXd uf_ieq_vec;
  // Set First Contact Constraint
  contact_list[0]->getRFConstraintMtx(Uf_);
  contact_list[0]->getRFConstraintVec(uf_ieq_vec_);

  // Contact Jacobian
  Jc_list_.clear();
  Eigen::MatrixXd Jc;
  contact_list[0]->getContactJacobian(Jc);
  Jc_list_.push_back(Jc);

  Jc_ = Jc;

  int dim_rf = contact_list[0]->getDim();
  int dim_rf_cstr = contact_list[0]->getDimRFConstratint();
  int dim_new_rf, dim_new_rf_cstr;

  for (int i(1); i < contact_list.size(); ++i) {
    contact_list[i]->getContactJacobian(Jc);
    dim_new_rf = contact_list[i]->getDim();
    dim_new_rf_cstr = contact_list[i]->getDimRFConstratint();

    // Stack Jc normally
    Jc_.conservativeResize(dim_rf + dim_new_rf, num_qdot_);
    Jc_.block(dim_rf, 0, dim_new_rf, num_qdot_) = Jc;

    // Store Jc
    Jc_list_.push_back(Jc);

    // Uf
    contact_list[i]->getRFConstraintMtx(Uf);
    Uf_.conservativeResize(dim_rf_cstr + dim_new_rf_cstr, dim_rf + dim_new_rf);
    Uf_.block(0, dim_rf, dim_rf_cstr, dim_new_rf).setZero();
    Uf_.block(dim_rf_cstr, 0, dim_new_rf_cstr, dim_rf).setZero();
    Uf_.block(dim_rf_cstr, dim_rf, dim_new_rf_cstr, dim_new_rf) = Uf;

    // Uf inequality vector
    contact_list[i]->getRFConstraintVec(uf_ieq_vec);
    uf_ieq_vec_.conservativeResize(dim_rf_cstr + dim_new_rf_cstr);
    uf_ieq_vec_.tail(dim_new_rf_cstr) = uf_ieq_vec;

    // Increase reaction force dimension
    dim_rf += dim_new_rf;
    dim_rf_cstr += dim_new_rf_cstr;
  }

  dim_contacts_ = dim_rf;
  dim_contact_constraints_ = dim_rf_cstr;
}

void IHWBC::prepareQPSizes() {
  n_quadprog_ = dim_dec_vars_;               // Number of decision Variables
  p_quadprog_ = dim_inequality_constraints_; // Number of Inequality constraints
  m_quadprog_ = 6;                           // Number of Equality Constraints

  qp_dec_vars_ = Eigen::VectorXd::Zero(n_quadprog_);
  qddot_result_ = Eigen::VectorXd::Zero(num_qdot_);
  Fr_result_ = Eigen::VectorXd::Zero(dim_contacts_);

  // Decision Variables
  x.resize(n_quadprog_);
  // Objective
  G.resize(n_quadprog_, n_quadprog_);
  g0.resize(n_quadprog_);
  // Equality Constraints
  CE.resize(n_quadprog_, m_quadprog_);
  ce0.resize(m_quadprog_);
  // Inequality Constraints
  CI.resize(n_quadprog_, p_quadprog_);
  ci0.resize(p_quadprog_);
}

void IHWBC::setQuadProgCosts(const Eigen::MatrixXd &P_cost,
                             const Eigen::VectorXd &v_cost) {
  // Set G
  for (int i = 0; i < n_quadprog_; i++) {
    for (int j = 0; j < n_quadprog_; j++) {
      G[i][j] = P_cost(i, j);
    }
  }
  // Set g0
  for (int i = 0; i < n_quadprog_; i++) {
    g0[i] = v_cost[i];
  }
}

void IHWBC::setEqualityConstraints(const Eigen::MatrixXd &Eq_mat,
                                   const Eigen::VectorXd &Eq_vec) {
  for (int i = 0; i < m_quadprog_; i++) {
    for (int j = 0; j < n_quadprog_; j++) {
      CE[j][i] = Eq_mat(i, j);
    }
    ce0[i] = Eq_vec[i];
  }
}

void IHWBC::setInequalityConstraints(const Eigen::MatrixXd &IEq_mat,
                                     const Eigen::VectorXd &IEq_vec) {
  for (int i = 0; i < p_quadprog_; ++i) {
    for (int j = 0; j < n_quadprog_; ++j) {
      CI[j][i] = IEq_mat(i, j);
    }
    ci0[i] = IEq_vec[i];
  }
}

void IHWBC::solveQP() {
  double qp_result = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);

  // Populate qd result from QP answer
  for (int i = 0; i < n_quadprog_; i++) {
    qp_dec_vars_[i] = x[i];
  }
  // Store results
  qddot_result_ = qp_dec_vars_.head(num_qdot_);
  Fr_result_ = qp_dec_vars_.tail(dim_contacts_);

  // myUtils::pretty_print(qp_dec_vars_, std::cout, "qp_dec_vars_");
  // myUtils::pretty_print(qddot_result_, std::cout, "qddot_result_");
  // myUtils::pretty_print(Fr_result_, std::cout, "Fr_result_");
}

/*

Eigen::MatrixXd dyn_CE(dim_eq_cstr_, dim_opt_);
Eigen::VectorXd dyn_ce0(dim_eq_cstr_);

Aqddot + b + g = Jc^T Fr

Sf(Aqddot + b + g - Jc^T Fr = 0)

Sf*A*qddot + Sf*(b+g) - (Sf*Jc^T) Fr = 0)

Sf*A*qddot + Sf*(b+g) - (Sf*Jc^T) Fr = 0)

[Sf*A - Sf*Jc^T] [qddot, Fr] = 0


*/
