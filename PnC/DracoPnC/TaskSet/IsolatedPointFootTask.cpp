#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <Configuration.h>
#include <Utils/IO/IOUtilities.hpp>

IsolatedPointFootTask::IsolatedPointFootTask(RobotSystem* robot,
                           const std::string & _link_name):Task(robot, 3)
{
    myUtils::pretty_constructor(3, _link_name + " Task");

    for (int i = 0; i < dim_task_; ++i) {
        kp_[i] = 150.;
        kd_[i] = 10.;
    }

    Jt_ = Eigen::MatrixXd::Zero(dim_task_, robot_->getNumDofs());
    JtDotQdot_ = Eigen::VectorXd::Zero(dim_task_);
    link_name_ = _link_name;
}

IsolatedPointFootTask::~IsolatedPointFootTask(){}

bool IsolatedPointFootTask::_UpdateCommand(const Eigen::VectorXd & _pos_des,
                                  const Eigen::VectorXd & _vel_des,
                                  const Eigen::VectorXd & _acc_des) {

    vel_des = _vel_des; acc_des = _acc_des;
    Eigen::VectorXd vel_act = Eigen::VectorXd::Zero(dim_task_);
    if (link_name_ == "rFootCenter") {
        pos_err = _pos_des -
            robot_->getBodyNodeCoMIsometry(DracoBodyNode::rFootCenter).translation();
        vel_act = robot_->
            getBodyNodeCoMSpatialVelocity(DracoBodyNode::rFootCenter).tail(3);
    } else if (link_name_ == "lFootCenter") {
         pos_err = _pos_des -
            robot_->getBodyNodeCoMIsometry(DracoBodyNode::lFootCenter).translation();
        vel_act = robot_->
            getBodyNodeCoMSpatialVelocity(DracoBodyNode::lFootCenter).tail(3);
    } else {
        std::cout << "Wrong Name" << std::endl;
        exit(0);
    }

    // op_cmd
    for(int i(0); i < dim_task_; ++i){
        op_cmd[i] = acc_des[i]
            + kp_[i] * pos_err[i]
            + kd_[i] * (vel_des[i] - vel_act[i]);
    }
    return true;
}

bool IsolatedPointFootTask::_UpdateTaskJacobian(){
    if (link_name_ == "rFootCenter") {
        Jt_ = (robot_->
                getBodyNodeCoMJacobian(DracoBodyNode::rFootCenter)).block(
                3, 0, dim_task_, robot_->getNumDofs());
    } else if (link_name_ == "lFootCenter") {
        Jt_ = (robot_->
                getBodyNodeCoMJacobian(DracoBodyNode::lFootCenter)).block(
                3, 0, dim_task_, robot_->getNumDofs());
    } else {
        std::cout << "Wrong Name" << std::endl;
        exit(0);
    }
    // isolate virtual joint
    Jt_.block(0, 0, 3, robot_->getNumVirtualDofs()) =
        Eigen::MatrixXd::Zero(3, robot_->getNumVirtualDofs());

    return true;
}

bool IsolatedPointFootTask::_UpdateTaskJDotQdot(){
    if (link_name_ == "rFootCenter") {
        JtDotQdot_ = robot_->
            getBodyNodeCoMJacobianDot(DracoBodyNode::rFootCenter).block(3, 0, dim_task_, robot_->getNumDofs()) * robot_->getQdot();
    } else if (link_name_ == "lFootCenter") {
        JtDotQdot_ = robot_->
            getBodyNodeCoMJacobianDot(DracoBodyNode::lFootCenter).block(3, 0, dim_task_, robot_->getNumDofs()) * robot_->getQdot();
    } else {
        std::cout << "Wrong Name" << std::endl;
        exit(0);
    }

    return true;
}
