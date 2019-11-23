#include <Configuration.h>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <Utils/IO/IOUtilities.hpp>

BodyRxRyZTask::BodyRxRyZTask(RobotSystem* robot) : Task(robot, 3) {
    myUtils::pretty_constructor(3, "Body Rx Ry Z Task");

    Jt_ = Eigen::MatrixXd::Zero(dim_task_, robot_->getNumDofs());
    JtDotQdot_ = Eigen::VectorXd::Zero(dim_task_);
}

BodyRxRyZTask::~BodyRxRyZTask() {}

bool BodyRxRyZTask::_UpdateCommand(const Eigen::VectorXd& _pos_des,
                                   const Eigen::VectorXd& _vel_des,
                                   const Eigen::VectorXd& _acc_des) {
    Eigen::Quaternion<double> des_ori(_pos_des[0], _pos_des[1], _pos_des[2],
                                      _pos_des[3]);
    Eigen::Quaternion<double> ori_act(
        robot_->getBodyNodeCoMIsometry(DracoBodyNode::Torso).linear());
    Eigen::Quaternion<double> quat_ori_err;
    quat_ori_err = des_ori * ori_act.inverse();
    Eigen::Vector3d ori_err_so3;
    ori_err_so3 = dart::math::quatToExp(quat_ori_err);

    // (Rx, Ry)
    for (int i = 0; i < 2; ++i) {
        // pos_err[i] = myUtils::bind_half_pi(ori_err_so3[i]);
        pos_err[i] = (ori_err_so3[i]);
        vel_des[i] = _vel_des[i];
        acc_des[i] = _acc_des[i];
    }
    // (Z)
    pos_err[2] = _pos_des[6] - (robot_->getQ())[2];
    vel_des[2] = _vel_des[5];
    acc_des[2] = _acc_des[5];

    Eigen::VectorXd vel_act = Eigen::VectorXd::Zero(dim_task_);
    vel_act.head(2) = robot_->getBodyNodeCoMSpatialVelocity(DracoBodyNode::Torso).head(2);
    vel_act.tail(1) = robot_->getBodyNodeCoMSpatialVelocity(DracoBodyNode::Torso).tail(1);

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

bool BodyRxRyZTask::_UpdateTaskJacobian() {
    // Eigen::MatrixXd Jtmp = robot_->getBodyNodeJacobian(DracoBodyNode::Torso);
    Eigen::MatrixXd Jtmp = robot_->getBodyNodeCoMJacobian(DracoBodyNode::Torso);
    // (Rx, Ry)
    Jt_.block(0, 0, 2, robot_->getNumDofs()) =
        Jtmp.block(0, 0, 2, robot_->getNumDofs());
    // (Z)
    Jt_(2, 2) = 1.0;
    // Jt_.block(2, 0, 1, robot_->getNumDofs()) = Jtmp.block(5, 0, 1,
    // robot_->getNumDofs());

    return true;
}

bool BodyRxRyZTask::_UpdateTaskJDotQdot() {
    JtDotQdot_.setZero();
    return true;
}
