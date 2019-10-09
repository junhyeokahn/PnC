#include <Configuration.h>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <Utils/IO/IOUtilities.hpp>

BodyRxRyTask::BodyRxRyTask(RobotSystem* robot) : Task(robot, 2) {
    myUtils::pretty_constructor(3, "Body Rx Ry Task");

    Jt_ = Eigen::MatrixXd::Zero(dim_task_, robot_->getNumDofs());
    JtDotQdot_ = Eigen::VectorXd::Zero(dim_task_);
}

BodyRxRyTask::~BodyRxRyTask() {}

bool BodyRxRyTask::_UpdateCommand(const Eigen::VectorXd& _pos_des,
                                   const Eigen::VectorXd& _vel_des,
                                   const Eigen::VectorXd& _acc_des) {
    Eigen::Quaternion<double> des_ori(_pos_des[0], _pos_des[1], _pos_des[2],
                                      _pos_des[3]);
    Eigen::Quaternion<double> ori_act(
        robot_->getBodyNodeCoMIsometry(DracoBodyNode::Torso).linear());
    //Eigen::MatrixXd ori_act_1(
        //robot_->getBodyNodeCoMIsometry(DracoBodyNode::Torso).linear());
    //std::cout << "==============="  << std::endl;
    //myUtils::pretty_print(ori_act, std::cout, "quatmatrix");
    //std::cout << "==============="  << std::endl;
    //myUtils::pretty_print(ori_act_1, std::cout, "quatmatrix");
    Eigen::Quaternion<double> quat_ori_err;
    quat_ori_err = des_ori * ori_act.inverse();
    Eigen::Vector3d ori_err_so3;
    ori_err_so3 = dart::math::quatToExp(quat_ori_err);

    // (Rx, Ry)
        // pos_err[i] = myUtils::bind_half_pi(ori_err_so3[i]);
    //for (int i = 0; i < 2; ++i) {
        //pos_err[i] = (ori_err_so3[i]);
        //vel_des[i] = _vel_des[i];
        //acc_des[i] = _acc_des[i];
    //}

        // (Ry,Rz)
        //pos_err[0] = (ori_err_so3[1]);
        //vel_des[0] = _vel_des[1];
        //acc_des[0] = _acc_des[1];

        //pos_err[1] = (ori_err_so3[2]);
        //vel_des[1] = _vel_des[2];
        //acc_des[1] = _acc_des[2];

    // (Rx,Rz)
        pos_err[0] = (ori_err_so3[0]);
        vel_des[0] = _vel_des[0];
        acc_des[0] = _acc_des[0];

        pos_err[1] = (ori_err_so3[2]);
        vel_des[1] = _vel_des[2];
        acc_des[1] = _acc_des[2];
    // (Z)
    //pos_err[2] = _pos_des[6] - (robot_->getQ())[2];
    //vel_des[2] = _vel_des[5];
    //acc_des[2] = _acc_des[5];

    // myUtils::pretty_print(des_ori, std::cout, "ori_des");
    // myUtils::pretty_print(ori_act, std::cout, "ori_act");
    // myUtils::pretty_print(pos_err, std::cout, "pos_err in bodyrpz");

    return true;
}

bool BodyRxRyTask::_UpdateTaskJacobian() {
    // Eigen::MatrixXd Jtmp = robot_->getBodyNodeJacobian(DracoBodyNode::Torso);
    Eigen::MatrixXd Jtmp = robot_->getBodyNodeCoMJacobian(DracoBodyNode::Torso);
    // (Rx, Ry)
    //Jt_.block(0, 0, 2, robot_->getNumDofs()) =
        //Jtmp.block(0, 0, 2, robot_->getNumDofs());

    // (Ry, Rz)
    //Jt_.block(0, 0, 1, robot_->getNumDofs()) =
        //Jtmp.block(1, 0, 1, robot_->getNumDofs());
    //Jt_.block(1, 0, 1, robot_->getNumDofs()) =
        //Jtmp.block(2, 0, 1, robot_->getNumDofs());

    // (Rx, Rz)
    Jt_.block(0, 0, 1, robot_->getNumDofs()) =
        Jtmp.block(0, 0, 1, robot_->getNumDofs());
    Jt_.block(1, 0, 1, robot_->getNumDofs()) =
        Jtmp.block(2, 0, 1, robot_->getNumDofs());
    // Jt_.block(2, 0, 1, robot_->getNumDofs()) = Jtmp.block(5, 0, 1,
    // robot_->getNumDofs());
    return true;
}

bool BodyRxRyTask::_UpdateTaskJDotQdot() {
    JtDotQdot_.setZero();
    return true;
}
