#include <Configuration.h>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <Utils/IO/IOUtilities.hpp>

IsolatedPointFootTask::IsolatedPointFootTask(RobotSystem* robot, int _link_idx)
    : Task(robot, 3) {
    myUtils::pretty_constructor(3, "Isolated Point Foot Task");

    for (int i = 0; i < dim_task_; ++i) {
        kp_[i] = 150.;
        kd_[i] = 10.;
    }

    Jt_ = Eigen::MatrixXd::Zero(dim_task_, robot_->getNumDofs());
    JtDotQdot_ = Eigen::VectorXd::Zero(dim_task_);
    link_idx_ = _link_idx;
}

IsolatedPointFootTask::~IsolatedPointFootTask() {}

bool IsolatedPointFootTask::_UpdateCommand(const Eigen::VectorXd& _pos_des,
                                           const Eigen::VectorXd& _vel_des,
                                           const Eigen::VectorXd& _acc_des) {
    vel_des = _vel_des;
    acc_des = _acc_des;
    Eigen::VectorXd vel_act = Eigen::VectorXd::Zero(dim_task_);
    pos_err =
        _pos_des - robot_->getBodyNodeCoMIsometry(link_idx_).translation();
    vel_act = robot_->getBodyNodeCoMSpatialVelocity(link_idx_).tail(3);

    // op_cmd
    for (int i(0); i < dim_task_; ++i) {
        op_cmd[i] = acc_des[i] + kp_[i] * pos_err[i] +
                    kd_[i] * (vel_des[i] - vel_act[i]);
    }
    return true;
}

bool IsolatedPointFootTask::_UpdateTaskJacobian() {
    Jt_ = (robot_->getBodyNodeCoMJacobian(link_idx_))
              .block(3, 0, dim_task_, robot_->getNumDofs());
    // isolate virtual joint
    Jt_.block(0, 0, 3, robot_->getNumVirtualDofs()) =
        Eigen::MatrixXd::Zero(3, robot_->getNumVirtualDofs());

    return true;
}

bool IsolatedPointFootTask::_UpdateTaskJDotQdot() {
    JtDotQdot_ = robot_->getBodyNodeCoMJacobianDot(link_idx_).block(
                     3, 0, dim_task_, robot_->getNumDofs()) *
                 robot_->getQdot();

    return true;
}
