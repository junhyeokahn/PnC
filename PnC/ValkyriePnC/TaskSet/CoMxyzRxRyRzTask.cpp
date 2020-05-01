#include <PnC/ValkyriePnC/TaskSet/TaskSet.hpp>
#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>
#include <Configuration.h>
#include <Utils/IO/IOUtilities.hpp>

CoMxyzRxRyRzTask::CoMxyzRxRyRzTask(RobotSystem* robot):Task(robot, 6)
{
    myUtils::pretty_constructor(3, "COM XYZRxRyRz Task");
    Jt_ = Eigen::MatrixXd::Zero(dim_task_, robot_->getNumDofs());
    JtDotQdot_ = Eigen::VectorXd::Zero(dim_task_);
}

CoMxyzRxRyRzTask::~CoMxyzRxRyRzTask(){}

bool CoMxyzRxRyRzTask::_UpdateCommand(const Eigen::VectorXd & _pos_des,
                                      const Eigen::VectorXd & _vel_des,
                                      const Eigen::VectorXd & _acc_des) {

    Eigen::Quaternion<double> des_ori(_pos_des[0], _pos_des[1], _pos_des[2], _pos_des[3]);
    Eigen::Quaternion<double> ori_act(robot_->getBodyNodeCoMIsometry(ValkyrieBodyNode::pelvis).linear());
    Eigen::Quaternion<double> quat_ori_err;
    quat_ori_err = des_ori * ori_act.inverse();
    Eigen::Vector3d ori_err_so3;
    ori_err_so3 = dart::math::quatToExp(quat_ori_err);

    // (Rx, Ry, Rz)
    for (int i = 0; i < 3; ++i) {
        //pos_err[i] = myUtils::bind_half_pi(ori_err_so3[i]);
        pos_err[i] = (ori_err_so3[i]);
        vel_des[i] = _vel_des[i];
        acc_des[i] = _acc_des[i];
    }
    //vel_act.head(3) = robot_->getBodyNodeCoMSpatialVelocity(DracoBodyNode::Torso).head(3);

    // (X, Y, Z)
    for (int i = 0; i < 3; ++i) {
        pos_err[i+3] = _pos_des[i+4] - robot_->getCoMPosition()[i];
        vel_des[i+3] = _vel_des[i+3];
        acc_des[i+3] = _acc_des[i+3];
    }

    // com_vel_act
    Eigen::Vector3d com_vel_act = robot_->getCoMVelocity();

    // op_cmd
    for (int i(0); i < 3; ++i) {
        op_cmd[i+3] = acc_des[i+3] + kp_[i+3] * pos_err[i+3] +
                    kd_[i+3] * (vel_des[i+3] - com_vel_act[i]);
    }

    //myUtils::pretty_print(des_ori, std::cout, "ori_des");
    //myUtils::pretty_print(ori_act, std::cout, "ori_act");
    //myUtils::pretty_print(pos_err, std::cout, "pos_err in com_lin_body_ori_task");

    return true;
}

bool CoMxyzRxRyRzTask::_UpdateTaskJacobian(){
    Eigen::MatrixXd Jtmp = robot_->getBodyNodeCoMJacobian(ValkyrieBodyNode::pelvis);
    // (Rx, Ry, Rz)
    Jt_.block(0 ,0 , 3, robot_->getNumDofs()) = Jtmp.block(0, 0, 3, robot_->getNumDofs());
    // (X, Y, Z)
    Jtmp = robot_->getCoMJacobian();
    Jt_.block(3, 0, 3, robot_->getNumDofs()) = Jtmp.block(3, 0, 3, robot_->getNumDofs());

    return true;
}

bool CoMxyzRxRyRzTask::_UpdateTaskJDotQdot(){
    JtDotQdot_.setZero();
    return true;
}
