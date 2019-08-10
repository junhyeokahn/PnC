#include <PnC/WBC/WBLC/KinWBC.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/pseudo_inverse.hpp>

KinWBC::KinWBC(const std::vector<bool>& act_joint)
    : num_act_joint_(0),
      threshold_(0.001)
// threshold_(0.005)
// threshold_(0.003)
{
    myUtils::pretty_constructor(3, "Kin WBC");
    num_qdot_ = act_joint.size();

    act_jidx_.clear();
    for (int i(0); i < num_qdot_; ++i) {
        if (act_joint[i]) {
            act_jidx_.push_back(i);
            ++num_act_joint_;
        }
    }
    // myUtils::pretty_print(act_jidx_, "act jidx");
    I_mtx = Eigen::MatrixXd::Identity(num_qdot_, num_qdot_);
}

bool KinWBC::FindConfiguration(const Eigen::VectorXd& curr_config,
                               const std::vector<Task*>& task_list,
                               const std::vector<ContactSpec*>& contact_list,
                               Eigen::VectorXd& jpos_cmd,
                               Eigen::VectorXd& jvel_cmd,
                               Eigen::VectorXd& jacc_cmd) {
    // printf("contact list size: %d\n", contact_list.size());
    // Contact Jacobian Setup
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

    // Pmx
    Eigen::MatrixXd Nc;
    _BuildProjectionMatrix(Jc, Nc);

    Eigen::VectorXd delta_q, qdot, qddot, JtDotQdot;
    Eigen::MatrixXd Jt, JtPre, JtPre_pinv, N_nx, N_pre;

    // First Task
    Task* task = task_list[0];
    task->getTaskJacobian(Jt);
    task->getTaskJacobianDotQdot(JtDotQdot);
    JtPre = Jt * Nc;
    _PseudoInverse(JtPre, JtPre_pinv);

    delta_q = JtPre_pinv * (task->pos_err);
    // myUtils::pretty_print(task->pos_err, std::cout, "com error");
    // myUtils::pretty_print(delta_q, std::cout, "delta_q");
    qdot = JtPre_pinv * (task->vel_des);
    qddot = JtPre_pinv * (task->acc_des - JtDotQdot);
    // qddot = JtPre_pinv * (task->op_cmd - JtDotQdot);

    Eigen::VectorXd prev_delta_q = delta_q;
    Eigen::VectorXd prev_qdot = qdot;
    Eigen::VectorXd prev_qddot = qddot;

    _BuildProjectionMatrix(JtPre, N_nx);
    N_pre = Nc * N_nx;

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

    for (int i(1); i < task_list.size(); ++i) {
        task = task_list[i];

        task->getTaskJacobian(Jt);
        task->getTaskJacobianDotQdot(JtDotQdot);
        JtPre = Jt * N_pre;

        _PseudoInverse(JtPre, JtPre_pinv);
        delta_q =
            prev_delta_q + JtPre_pinv * (task->pos_err - Jt * prev_delta_q);
        qdot = prev_qdot + JtPre_pinv * (task->vel_des - Jt * prev_qdot);
        qddot = prev_qddot +
                JtPre_pinv * (task->acc_des - JtDotQdot - Jt * prev_qddot);

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
    for (int i(0); i < num_act_joint_; ++i) {
        jpos_cmd[i] = curr_config[act_jidx_[i]] + delta_q[act_jidx_[i]];
        jvel_cmd[i] = qdot[act_jidx_[i]];
        jacc_cmd[i] = qddot[act_jidx_[i]];
    }
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
