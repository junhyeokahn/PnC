#include "WBC/KinWBC.hpp"
#include <Utilities/Utilities_print.h>
#include <Utilities/pseudoInverse.h>


KinWBC::KinWBC(int num_qdot)
    : threshold_(0.001), num_qdot_(num_qdot), num_act_joint_(num_qdot - 6) {
  I_mtx = Eigen::MatrixXd::Identity(num_qdot_, num_qdot_);
}


bool KinWBC::FindConfiguration(
    const Eigen::VectorXd& curr_config, const std::vector<Task*>& task_list,
    const std::vector<ContactSpec*>& contact_list, Eigen::VectorXd& jpos_cmd,
    Eigen::VectorXd& jvel_cmd) {

  // Contact Jacobian Setup
  Eigen::MatrixXd Nc(num_qdot_, num_qdot_); Nc.setIdentity();
  if(contact_list.size() > 0){
    Eigen::MatrixXd Jc, Jc_i;
    contact_list[0]->getContactJacobian(Jc);
    int num_rows = Jc.rows();

    for (int i(1); i < contact_list.size(); ++i) {
      contact_list[i]->getContactJacobian(Jc_i);
      int num_new_rows = Jc_i.rows();
      Jc.conservativeResize(num_rows + num_new_rows, num_qdot_);
      Jc.block(num_rows, 0, num_new_rows, num_qdot_) = Jc_i;
      num_rows += num_new_rows;
    }

    // Projection Matrix
    _BuildProjectionMatrix(Jc, Nc);
  }

  // First Task
  Eigen::VectorXd delta_q, qdot;
  Eigen::MatrixXd Jt, JtPre, JtPre_pinv, N_nx, N_pre;

  Task* task = task_list[0];
  task->getTaskJacobian(Jt);
  JtPre = Jt * Nc;
  _PseudoInverse(JtPre, JtPre_pinv);

  delta_q = JtPre_pinv * (task->getPosError());
  qdot = JtPre_pinv * (task->getDesVel());

  Eigen::VectorXd prev_delta_q = delta_q;
  Eigen::VectorXd prev_qdot = qdot;

  _BuildProjectionMatrix(JtPre, N_nx);
  N_pre = Nc * N_nx;

  for (int i(1); i < task_list.size(); ++i) {
    task = task_list[i];

    task->getTaskJacobian(Jt);
    JtPre = Jt * N_pre;

    _PseudoInverse(JtPre, JtPre_pinv);
    delta_q =
        prev_delta_q + JtPre_pinv * (task->getPosError() - Jt * prev_delta_q);
    qdot = prev_qdot + JtPre_pinv * (task->getDesVel() - Jt * prev_qdot);

    // For the next task
    _BuildProjectionMatrix(JtPre, N_nx);
    N_pre *= N_nx;
    prev_delta_q = delta_q;
    prev_qdot = qdot;
  }
  for (int i(0); i < num_act_joint_; ++i) {
    jpos_cmd[i] = curr_config[i + 6] + delta_q[i + 6];
    jvel_cmd[i] = qdot[i + 6];
  }
  return true;
}


void KinWBC::_BuildProjectionMatrix(const Eigen::MatrixXd& J, Eigen::MatrixXd& N) {
  Eigen::MatrixXd J_pinv;
  _PseudoInverse(J, J_pinv);
  N = I_mtx - J_pinv * J;
}


void KinWBC::_PseudoInverse(const Eigen::MatrixXd J, Eigen::MatrixXd& Jinv) {
  pseudoInverse(J, threshold_, Jinv);
}

// template class KinWBC<float>;
// template class KinWBC<double>;
