#include <Configuration.h>
#include <PnC/ScorpioPnC/ScorpioDefinition.hpp>
#include <PnC/ScorpioPnC/TaskSet/SuctionGripperTask.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

SuctionGripperTask::SuctionGripperTask(RobotSystem* robot) : Task(robot, 2) {
    myUtils::pretty_constructor(3, "Suction Gripper Task");

    Jt_ = Eigen::MatrixXd::Zero(dim_task_, robot_->getNumDofs());
    JtDotQdot_ = Eigen::VectorXd::Zero(dim_task_);
}

SuctionGripperTask::~SuctionGripperTask() {}

bool SuctionGripperTask::_UpdateCommand(const Eigen::VectorXd& _pos_des,
                                   const Eigen::VectorXd& _vel_des,
                                   const Eigen::VectorXd& _acc_des) {
    Eigen::Quaternion<double> des_ori(_pos_des[0], _pos_des[1], _pos_des[2],
                                      _pos_des[3]);
    Eigen::Quaternion<double> ori_act(
        robot_->getBodyNodeCoMIsometry(ScorpioBodyNode::end_effector).linear());
    //TEST
    des_ori = myUtils::bind_qaut_pi(des_ori);
    ori_act = myUtils::bind_qaut_pi(ori_act);
    //TEST
    //std::cout << "++++++++++++++++++++++++" << std::endl;
    //myUtils::pretty_print(des_ori, std::cout, "des_ori");
    //myUtils::pretty_print(ori_act, std::cout, "act_ori");
    //
    Eigen::Quaternion<double> quat_ori_err;
    quat_ori_err = des_ori * ori_act.inverse();
    Eigen::Vector3d ori_err_so3;
    ori_err_so3 = dart::math::quatToExp(quat_ori_err);

    // (Rx, Ry)
    for (int i = 0; i < dim_task_; ++i) {
        //pos_err[i] = myUtils::bind_half_pi(ori_err_so3[i]);
        pos_err[i] = (ori_err_so3[i]);
        vel_des[i] = _vel_des[i];
        acc_des[i] = _acc_des[i];
    }

    Eigen::VectorXd vel_act = robot_->getBodyNodeSpatialVelocity(ScorpioBodyNode::end_effector).head(3);
    for (int i = 0; i < dim_task_; ++i) {
        op_cmd[i] = acc_des[i]
            + kp_[i] * pos_err[i]
            + kd_[i] * (vel_des[i] - vel_act[i]);
    }

    // (Ry,Rz)
    //pos_err[0] = (ori_err_so3[1]);
    //vel_des[0] = _vel_des[1];
    //acc_des[0] = _acc_des[1];

    //pos_err[1] = (ori_err_so3[2]);
    //vel_des[1] = _vel_des[2];
    //acc_des[1] = _acc_des[2];

    // (Rx,Rz)
    //pos_err[0] = (ori_err_so3[0]);
    //vel_des[0] = _vel_des[0];
    //acc_des[0] = _acc_des[0];

    //pos_err[1] = (ori_err_so3[2]);
    //vel_des[1] = _vel_des[2];
    //acc_des[1] = _acc_des[2];

    // myUtils::pretty_print(des_ori, std::cout, "ori_des");
    // myUtils::pretty_print(ori_act, std::cout, "ori_act");
    // myUtils::pretty_print(pos_err, std::cout, "pos_err in bodyrpz");

    return true;
}

bool SuctionGripperTask::_UpdateTaskJacobian() {
    Eigen::MatrixXd Jtmp = robot_->getBodyNodeCoMJacobian(ScorpioBodyNode::end_effector);
    // (Rx, Ry)
    Jt_.block(0, 0, 2, robot_->getNumDofs()) =
        Jtmp.block(0, 0, 2, robot_->getNumDofs());

    // (Ry, Rz)
    //Jt_.block(0, 0, 1, robot_->getNumDofs()) =
        //Jtmp.block(1, 0, 1, robot_->getNumDofs());
    //Jt_.block(1, 0, 1, robot_->getNumDofs()) =
        //Jtmp.block(2, 0, 1, robot_->getNumDofs());

    // (Rx, Rz)
    //Jt_.block(0, 0, 1, robot_->getNumDofs()) =
        //Jtmp.block(0, 0, 1, robot_->getNumDofs());
    //Jt_.block(1, 0, 1, robot_->getNumDofs()) =
        //Jtmp.block(2, 0, 1, robot_->getNumDofs());

    // Jt_.block(2, 0, 1, robot_->getNumDofs()) = Jtmp.block(5, 0, 1,
    // robot_->getNumDofs());
    return true;
}

bool SuctionGripperTask::_UpdateTaskJDotQdot() {
    JtDotQdot_.setZero();
    return true;
}
