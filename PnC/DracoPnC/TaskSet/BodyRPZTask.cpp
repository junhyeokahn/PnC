#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <Configuration.h>
#include <Utils/Utilities.hpp>

BodyRPZTask::BodyRPZTask(RobotSystem* robot):Task(robot, 3)
{
    Jt_ = Eigen::MatrixXd::Zero(dim_task_, robot_->getNumDofs());
    JtDotQdot_ = Eigen::VectorXd::Zero(dim_task_);
}

BodyRPZTask::~BodyRPZTask(){}

bool BodyRPZTask::_UpdateCommand(const Eigen::VectorXd & _pos_des,
        const Eigen::VectorXd & _vel_des,
        const Eigen::VectorXd & _acc_des) {

    Eigen::Quaternion<double> des_ori(_pos_des[0], _pos_des[1], _pos_des[2], _pos_des[3]);
    Eigen::Quaternion<double> ori_act(robot_->getBodyNodeCoMIsometry("torso").linear());
    Eigen::Quaternion<double> quat_ori_err;
    quat_ori_err = des_ori * ori_act.inverse();
    Eigen::Vector3d ori_err_so3;
    ori_err_so3 = dart::math::quatToExp(quat_ori_err);

    // (Rx, Ry)
    for (int i = 0; i < 2; ++i) {
        //pos_err[i] = myUtils::bind_half_pi(ori_err_so3[i]);
        pos_err[i] = (ori_err_so3[i]);
        vel_des[i] = _vel_des[i];
        acc_des[i] = _acc_des[i];
    }
    // (Z)
    pos_err[2] = _pos_des[6] - (robot_->getQ())[2];
    vel_des[2] = vel_des[5];
    acc_des[2] = acc_des[5];

    //myUtils::pretty_print(des_ori, std::cout, "ori_des");
    //myUtils::pretty_print(ori_act, std::cout, "ori_act");
    //myUtils::pretty_print(pos_err, std::cout, "pos_err in bodyrpz");

    return true;
}

bool BodyRPZTask::_UpdateTaskJacobian(){
    //Eigen::MatrixXd Jtmp = robot_->getBodyNodeJacobian("torso");
    Eigen::MatrixXd Jtmp = robot_->getBodyNodeCoMJacobian("torso");
    // (Rx, Ry)
    Jt_.block(0 ,0 , 2, robot_->getNumDofs()) = Jtmp.block(0, 0, 2, robot_->getNumDofs());
    // (Z)
    Jt_(2, 2) = 1.0;
    //Jt_.block(2, 0, 1, robot_->getNumDofs()) = Jtmp.block(5, 0, 1, robot_->getNumDofs());

    return true;
}

bool BodyRPZTask::_UpdateTaskJDotQdot(){
    JtDotQdot_.setZero();
    return true;
}
