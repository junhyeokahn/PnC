#include <Configuration.h>
#include <PnC/ScorpioPnC/ScorpioDefinition.hpp>
#include <PnC/ScorpioPnC/TaskSet/SuctionGripperTask.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

#include <Utils/IO/DataManager.hpp>

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
    
    Eigen::Quaternion<double> quat_ori_err;
    quat_ori_err = des_ori * ori_act.inverse();
    Eigen::Vector3d ori_err_so3;
    ori_err_so3 = dart::math::quatToExp(quat_ori_err);

    Eigen::Vector3d des_ori_data = dart::math::quatToExp(des_ori);
    Eigen::Vector3d act_ori_data = dart::math::quatToExp(ori_act);

    //Printout
    //myUtils::pretty_print(des_ori_data, std::cout, "des_ori");
    //myUtils::pretty_print(act_ori_data, std::cout, "act_ori");
    //myUtils::pretty_print(ori_err_so3, std::cout, "ori_err");

    //op_cmd.head(3) = robot_->getBodyNodeCoMIsometry("end_effector").linear().transpose() * acc_des.head(3)
                //+ robot_->getBodyNodeCoMIsometry("end_effector").linear().transpose() * ori_err_so3 
                //+ robot_->getBodyNodeCoMIsometry("end_effector").linear().transpose() * 

    Eigen::MatrixXd R_tr = robot_->getBodyNodeCoMIsometry("end_effector").linear().transpose();
    ori_err_so3 = R_tr * ori_err_so3;
    Eigen::VectorXd loc_vel = R_tr * _vel_des;
    Eigen::VectorXd loc_acc = R_tr * _acc_des;

    // (Rx, Ry)
    for (int i = 0; i < dim_task_; ++i) {
        pos_err[i] = (ori_err_so3[i]);
        //vel_des[i] = _vel_des[i];
        //acc_des[i] = _acc_des[i];
        vel_des[i] = loc_vel[i];
        acc_des[i] = loc_acc[i];
    }

    Eigen::VectorXd vel_act = robot_->getBodyNodeSpatialVelocity(ScorpioBodyNode::end_effector).head(3);
    Eigen::VectorXd loc_vel_act =  R_tr * vel_act;
    for (int i = 0; i < dim_task_; ++i) {
        op_cmd[i] =  acc_des[i+1]
            + kp_[i] * pos_err[i+1]
            + kd_[i] * (vel_des[i+1] - loc_vel_act[i+1]);
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
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(6, 6);
    R.block(0, 0, 3, 3) = robot_->getBodyNodeCoMIsometry("end_effector").linear().transpose();
    R.block(3, 3, 3, 3) = robot_->getBodyNodeCoMIsometry("end_effector").linear().transpose();
    Jtmp =  R * Jtmp;
    // local (Ry, Rz)
    Jt_.block(0, 0, 2, robot_->getNumDofs()) =
        Jtmp.block(1, 0, 2, robot_->getNumDofs());

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
