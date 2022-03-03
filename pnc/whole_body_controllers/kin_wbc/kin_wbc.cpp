#include "pnc/whole_body_controllers/kin_wbc/kin_wbc.hpp"

KinWBC::KinWBC(const Eigen::MatrixXd &_sf) {
  n_floating_ = _sf.rows();
  n_qdot_ = _sf.cols();
}

bool KinWBC::solve(
    const Eigen::VectorXd &_curr_jpos, const std::vector<Task *> &_task_list,
    const std::vector<Contact *> &_contact_list,
    const std::vector<InternalConstraint *> &_internal_constraint_list,
    Eigen::VectorXd &_jpos_cmd, Eigen::VectorXd &_jvel_cmd,
    Eigen::VectorXd &_jacc_cmd) {

  // Internal Constraint
  Eigen::MatrixXd ji;
  bool b_internal_constraint;
  if (_internal_constraint_list.size() > 0) {
    b_internal_constraint = true;
    for (int i = 0; i < _internal_constraint_list.size(); ++i) {
      if (i == 0) {
        ji = _internal_constraint_list[i]->jacobian;
      } else {
        ji = util::vStack(ji, _internal_constraint_list[i]->jacobian);
      }
    }
  } else {
    b_internal_constraint = false;
  }

  // Contact
  Eigen::MatrixXd jc;
  bool b_contact_constraint;
  if (_contact_list.size() > 0) {
    b_contact_constraint = true;
    for (int i = 0; i < _contact_list.size(); ++i) {
      if (i == 0) {
        jc = _contact_list[i]->jacobian;
      } else {
        jc = util::vStack(jc, _contact_list[i]->jacobian);
      }
    }
  } else {
    b_contact_constraint = false;
  }

  // Projection
  Eigen::MatrixXd null_pre;
  if (b_internal_constraint) {
    if (b_contact_constraint) {
      // internal constraint (o)  contact constraint (o)
      Eigen::MatrixXd ji_null;
      _BuildProjectionMatrix(ji, ji_null);
      Eigen::MatrixXd jc_ji_null = jc * ji_null;
      _BuildProjectionMatrix(jc_ji_null, null_pre);
    } else {
      // internal constraint (o) contact constraint (x)
      _BuildProjectionMatrix(ji, null_pre);
    }
  } else {
    if (b_contact_constraint) {
      // internal constraint (x)  contact constraint (o)
      _BuildProjectionMatrix(jc, null_pre);
    } else {
      // internal constraint (x) contact constraint (x)
      null_pre = Eigen::MatrixXd::Identity(n_qdot_, n_qdot_);
    }
  }

  Eigen::VectorXd delta_q, qdot, qddot, JtDotQdot;
  Eigen::MatrixXd Jt, JtPre, JtPre_pinv, N_nx, N_pre;

  // First Task
  Task *task = _task_list[0];
  Jt = task->jacobian;
  JtDotQdot = task->jacobian_dot_q_dot;
  JtPre = Jt * null_pre;
  _PseudoInverse(JtPre, JtPre_pinv);

  delta_q = JtPre_pinv * (task->pos_err);
  qdot = JtPre_pinv * (task->vel_des);
  qddot = JtPre_pinv * (task->acc_des - JtDotQdot);
  // qddot = JtPre_pinv * (task->op_cmd - JtDotQdot);

  Eigen::VectorXd prev_delta_q = delta_q;
  Eigen::VectorXd prev_qdot = qdot;
  Eigen::VectorXd prev_qddot = qddot;

  _BuildProjectionMatrix(JtPre, N_nx);
  N_pre = null_pre * N_nx;

  // myUtils::color_print(myColor::Red, "======== 0 ========");
  // Eigen::VectorXd xdot_c = Jc * delta_q;
  // myUtils::pretty_print(xdot_c, std::cout, "contact vel");
  // myUtils::pretty_print(Jt, std::cout, "task Jt");
  // myUtils::pretty_print(Jc, std::cout, "Jc");
  // myUtils::pretty_print(Nc, std::cout, "Nc");
  // myUtils::pretty_print(JtPre, std::cout, "JtNc");
  // myUtils::pretty_print(JtPre_pinv, std::cout, "JtPre_inv");
  // myUtils::pretty_print(task->pos_err, std::cout, "pos_err");
  // myUtils::pretty_print(task->vel_des, std::cout, "vel_des");
  // myUtils::pretty_print(delta_q, std::cout, "delta q");
  // myUtils::pretty_print(qdot, std::cout, "qdot");
  // myUtils::pretty_print(qddot, std::cout, "qddot");
  // Eigen::MatrixXd test = Jt * N_pre;
  // myUtils::pretty_print(test, std::cout, "Jt1N1");
  // Eigen::JacobiSVD<Eigen::MatrixXd> svd1(
  // JtPre, Eigen::ComputeThinU | Eigen::ComputeThinV);
  // std::cout << "svd" << std::endl;
  // std::cout << svd1.singularValues() << std::endl;

  for (int i(1); i < _task_list.size(); ++i) {
    task = _task_list[i];

    Jt = task->jacobian;
    JtDotQdot = task->jacobian_dot_q_dot;
    JtPre = Jt * N_pre;

    _PseudoInverse(JtPre, JtPre_pinv);
    delta_q = prev_delta_q + JtPre_pinv * (task->pos_err - Jt * prev_delta_q);
    qdot = prev_qdot + JtPre_pinv * (task->vel_des - Jt * prev_qdot);
    qddot =
        prev_qddot + JtPre_pinv * (task->acc_des - JtDotQdot - Jt * prev_qddot);
    // qddot =
    // prev_qddot + JtPre_pinv * (task->op_cmd - JtDotQdot - Jt * prev_qddot);

    // myUtils::color_print(myColor::Red,
    //"======== " + std::to_string(i) + " ========");
    // myUtils::pretty_print(Jt, std::cout, "Jt");
    // myUtils::pretty_print(N_pre, std::cout, "N_pre");
    // myUtils::pretty_print(JtPre, std::cout, "JtPre");
    // myUtils::pretty_print(JtPre_pinv, std::cout, "JtPre_inv");
    // myUtils::pretty_print(task->pos_err, std::cout, "pos_err");
    // myUtils::pretty_print(task->vel_des, std::cout, "vel_des");
    // myUtils::pretty_print(delta_q, std::cout, "delta q");
    // myUtils::pretty_print(qdot, std::cout, "qdot");
    // xdot_c = Jc * delta_q;
    // myUtils::pretty_print(xdot_c, std::cout, "contact vel");
    // Eigen::JacobiSVD<Eigen::MatrixXd> svd2(
    // JtPre, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // std::cout << "svd" << std::endl;
    // std::cout << svd2.singularValues() << std::endl;

    // For the next task
    _BuildProjectionMatrix(JtPre, N_nx);
    N_pre *= N_nx;
    prev_delta_q = delta_q;
    prev_qdot = qdot;
    prev_qddot = qddot;
  }
  // xdot_c = Jc * delta_q;
  // myUtils::pretty_print(xdot_c, std::cout, "contact vel");
  // myUtils::pretty_print(delta_q, std::cout, "delta_q");
  for (int i(0); i < n_qdot_ - n_floating_; ++i) {
    _jpos_cmd[i] = _curr_jpos[i] + delta_q[n_floating_ + i];
    _jvel_cmd[i] = qdot[n_floating_ + i];
    _jacc_cmd[i] = qddot[n_floating_ + i];
  }
  return true;
}

void KinWBC::_BuildProjectionMatrix(const Eigen::MatrixXd &J,
                                    Eigen::MatrixXd &N) {
  Eigen::MatrixXd J_pinv;
  _PseudoInverse(J, J_pinv);
  N = Eigen::MatrixXd::Identity(n_qdot_, n_qdot_);
  -J_pinv *J;
}

void KinWBC::_PseudoInverse(const Eigen::MatrixXd J, Eigen::MatrixXd &Jinv) {
  util::PseudoInverse(J, threshold_, Jinv);
}
