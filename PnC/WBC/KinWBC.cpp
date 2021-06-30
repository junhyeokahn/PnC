#include <PnC/WBC/KinWBC.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/pseudo_inverse.hpp>

KinWBC::KinWBC(const std::vector<bool>& act_joint)
    : num_act_joint_(12), threshold_(0.001), num_qdot_(18) {
  myUtils::pretty_constructor(3, "Kin WBC");

  I_mtx = Eigen::MatrixXd::Identity(num_qdot_, num_qdot_);
}

bool KinWBC::FindConfiguration(const Eigen::VectorXd& curr_config,
                               const std::vector<Task*>& task_list,
                               const std::vector<ContactSpec*>& contact_list,
                               Eigen::VectorXd& jpos_cmd,
                               Eigen::VectorXd& jvel_cmd) {
  // Contact Jacobian Setup
  Eigen::MatrixXd Nc(num_qdot_, num_qdot_);
  Nc.setIdentity();
  // if(contact_list.size() > 0){
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
  // myUtils::pretty_print(Jc, std::cout, "Jc");
  // Projection Matrix
  _BuildProjectionMatrix(Jc, Nc);
  // }

  // First Task
  Eigen::VectorXd delta_q, qdot;
  Eigen::MatrixXd Jt, JtPre, JtPre_pinv, N_nx, N_pre;

  Task* task = task_list[0];
  task->getTaskJacobian(Jt);
  JtPre = Jt * Nc;
  _PseudoInverse(JtPre, JtPre_pinv);

  delta_q = JtPre_pinv * (task->pos_err);
  qdot = JtPre_pinv * (task->vel_des);

  Eigen::VectorXd prev_delta_q = delta_q;
  Eigen::VectorXd prev_qdot = qdot;

  _BuildProjectionMatrix(JtPre, N_nx);
  N_pre = Nc * N_nx;

  // std::cout << "Task list size = " << task_list.size() << std::endl;
  // myUtils::pretty_print(task->pos_err, std::cout, "[KinWBC] Task 1 Position
  // Error");
  // myUtils::pretty_print(task->vel_des, std::cout, "[KinWBC] Task 1 Vel Des");
  // myUtils::pretty_print(Jt, std::cout, "task Jt");
  // myUtils::pretty_print(Jc, std::cout, "Jc");
  // myUtils::pretty_print(Nc, std::cout, "Nc");
  // myUtils::pretty_print(JtPre, std::cout, "JtNc");
  // Eigen::JacobiSVD<Eigen::MatrixXd> svd(JtPre_pinv, Eigen::ComputeThinU |
  // Eigen::ComputeThinV); std::cout << "singular values: " <<
  // svd.singularValues() << std::endl; myUtils::pretty_print(JtPre_pinv,
  // std::cout, "JtPre_inv");
  // myUtils::pretty_print(delta_q, std::cout, "Task 1 delta q");
  // myUtils::pretty_print(qdot, std::cout, "qdot");

  for (int i(1); i < task_list.size(); ++i) {
    task = task_list[i];

    task->getTaskJacobian(Jt);
    JtPre = Jt * N_pre;

    _PseudoInverse(JtPre, JtPre_pinv);
    delta_q = prev_delta_q + JtPre_pinv * (task->pos_err - Jt * prev_delta_q);
    qdot = prev_qdot + JtPre_pinv * (task->vel_des - Jt * prev_qdot);

    // For the next task
    _BuildProjectionMatrix(JtPre, N_nx);
    N_pre *= N_nx;
    prev_delta_q = delta_q;
    prev_qdot = qdot;

    // myUtils::pretty_print(task->pos_err, std::cout, "[KinWBC] Task 2 Position
    // Error"); myUtils::pretty_print(delta_q, std::cout, "Task 2 delta q");
  }

  for (size_t i(0); i < num_act_joint_; ++i) {
    jpos_cmd[i] = curr_config[i + 6] + delta_q[i + 6];
    jvel_cmd[i] = qdot[i + 6];
  }
  Eigen::VectorXd temp;
  temp = Jc * qdot;
  // myUtils::pretty_print(temp, std::cout, "Jc * qdot_sol");
  // myUtils::pretty_print(jpos_cmd, std::cout, "jpos_cmd");
  // std::cout <<
  // "-------------------------------------------------------------" <<
  // std::endl;
  // Eigen::VectorXd xdot_c = Jc * delta_q;
  // myUtils::pretty_print(xdot_c, std::cout, "contact vel");
  return true;
}

void KinWBC::_BuildProjectionMatrix(const Eigen::MatrixXd& J,
                                    Eigen::MatrixXd& N) {
  Eigen::MatrixXd J_pinv;
  _PseudoInverse(J, J_pinv);
  N = I_mtx - J_pinv * J;
}

void KinWBC::_PseudoInverse(const Eigen::MatrixXd J, Eigen::MatrixXd& Jinv) {
  myUtils::pseudoInverse(J, threshold_, Jinv);

  // mx Lambda_inv = J * Ainv_ * J.transpose();
  // mx Lambda;
  // dynacore::pseudoInverse(Lambda_inv, threshold_, Lambda);
  // Jinv = Ainv_ * J.transpose() * Lambda;
}