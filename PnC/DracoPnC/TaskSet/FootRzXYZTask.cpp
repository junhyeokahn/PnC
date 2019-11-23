#include <Configuration.h>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <Utils/IO/IOUtilities.hpp>

FootRzXYZTask::FootRzXYZTask(RobotSystem* robot, int _link_idx)
    : Task(robot, 4) {
    myUtils::pretty_constructor(3, "Rz X Y Z Foot Task");

    Jt_ = Eigen::MatrixXd::Zero(dim_task_, robot_->getNumDofs());
    JtDotQdot_ = Eigen::VectorXd::Zero(dim_task_);
    link_idx_ = _link_idx;
}

FootRzXYZTask::~FootRzXYZTask() {}

bool FootRzXYZTask::_UpdateCommand(const Eigen::VectorXd& _pos_des,
                                   const Eigen::VectorXd& _vel_des,
                                   const Eigen::VectorXd& _acc_des) {
    Eigen::Quaternion<double> des_ori(_pos_des[0], _pos_des[1], _pos_des[2],
                                      _pos_des[3]);
    Eigen::Quaternion<double> ori_act(
        robot_->getBodyNodeIsometry(link_idx_).linear());
    Eigen::Quaternion<double> quat_ori_err = des_ori * ori_act.inverse();
    Eigen::Vector3d ori_err_so3 = dart::math::quatToExp(quat_ori_err);

    // (Rz)
    pos_err[0] = ori_err_so3[2];
    vel_des[0] = _vel_des[2];
    acc_des[0] = _acc_des[2];
    // (x, y, z)
    Eigen::VectorXd pos_act =
        robot_->getBodyNodeIsometry(link_idx_).translation();
    for (int i = 0; i < 3; ++i) {
        pos_err[i + 1] = _pos_des[i + 4] - pos_act[i];
        vel_des[i + 1] = _vel_des[i + 3];
        acc_des[i + 1] = _acc_des[i + 3];
    }

    Eigen::VectorXd vel_act = Eigen::VectorXd::Zero(dim_task_);
    vel_act.head(1) = robot_->getBodyNodeCoMSpatialVelocity(DracoBodyNode::Torso).head(1);
    vel_act.tail(3) = robot_->getBodyNodeCoMSpatialVelocity(DracoBodyNode::Torso).tail(3);

    // op_cmd
    for (int i(0); i < dim_task_; ++i) {
        op_cmd[i] = acc_des[i] + kp_[i] * pos_err[i] +
                    kd_[i] * (vel_des[i] - vel_act[i]);
    }

    // myUtils::pretty_print(des_ori, std::cout, "ori_des");
    // myUtils::pretty_print(ori_act, std::cout, "ori_act");
    // myUtils::pretty_print(pos_err, std::cout, "pos_err in bodyrpz");

    return true;
}

bool FootRzXYZTask::_UpdateTaskJacobian() {
    Eigen::MatrixXd Jtmp = robot_->getBodyNodeJacobian(link_idx_);
    // Rz
    Jt_.block(0, 0, 1, robot_->getNumDofs()) =
        Jtmp.block(2, 0, 1, robot_->getNumDofs());
    // (x, y, z)
    Jt_.block(1, 0, 3, robot_->getNumDofs()) =
        Jtmp.block(3, 0, 3, robot_->getNumDofs());
    // isolate virtual joint
    Jt_.block(0, 0, dim_task_, robot_->getNumVirtualDofs()) =
        Eigen::MatrixXd::Zero(dim_task_, robot_->getNumVirtualDofs());

    return true;
}

bool FootRzXYZTask::_UpdateTaskJDotQdot() {
    Eigen::VectorXd v_tmp =
        robot_->getBodyNodeJacobianDot(link_idx_) * robot_->getQdot();
    JtDotQdot_.segment(0, 1) = v_tmp.segment(2, 1);
    JtDotQdot_.tail(3) = v_tmp.tail(3);

    JtDotQdot_.setZero();
    return true;
}
