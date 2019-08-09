#include <Configuration.h>
#include <PnC/AtlasPnC/AtlasDefinition.hpp>
#include <PnC/AtlasPnC/TaskSet/TaskSet.hpp>
#include <Utils/IO/IOUtilities.hpp>

CoMTask::CoMTask(RobotSystem* robot) : Task(robot, 3) {
    myUtils::pretty_constructor(3, "CoM Task");

    Jt_ = Eigen::MatrixXd::Zero(dim_task_, robot_->getNumDofs());
    JtDotQdot_ = Eigen::VectorXd::Zero(dim_task_);
}

CoMTask::~CoMTask() {}

bool CoMTask::_UpdateCommand(const Eigen::VectorXd& _pos_des,
                             const Eigen::VectorXd& _vel_des,
                             const Eigen::VectorXd& _acc_des) {
    // (CoM x,y,z)
    for(int i(0); i < 3; i++) {
    pos_err[i] = _pos_des[i] - (robot_->getCoMPosition())[i];
    vel_des[i] = _vel_des[i];
    acc_des[i] = _acc_des[i];
    }

    return true;
}

bool CoMTask::_UpdateTaskJacobian() {
    // Eigen::MatrixXd Jtmp = robot_->getBodyNodeJacobian(DracoBodyNode::Torso);
    Eigen::MatrixXd Jtmp = robot_->getCoMJacobian();
    // (x, y, z) 
    Jt_ = Jtmp.block(3, 0, 3, robot_->getNumDofs());
    // Jt_.block(2, 0, 1, robot_->getNumDofs()) = Jtmp.block(5, 0, 1,
    // robot_->getNumDofs());

    return true;
}

bool CoMTask::_UpdateTaskJDotQdot() {
    JtDotQdot_.setZero();
    return true;
}
