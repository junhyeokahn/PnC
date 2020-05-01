#include <PnC/ValkyriePnC/TaskSet/TaskSet.hpp>
#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>
#include <Configuration.h>
#include <Utils/IO/IOUtilities.hpp>

SelectedJointTasks::SelectedJointTasks(RobotSystem* robot, const std::vector<int> & joint_indices):Task(robot, joint_indices.size())
{
    myUtils::pretty_constructor(3, "Selected Joint Tasks");
    Jt_ = Eigen::MatrixXd::Zero(dim_task_, robot_->getNumDofs());

    joint_indices_ = joint_indices;
    // Set element of corresponding joint index
    for(int i = 0; i < joint_indices_.size(); i++){
        Jt_(i, joint_indices_[i]) = 1.0;
    }
    JtDotQdot_ = Eigen::VectorXd::Zero(dim_task_);
}

SelectedJointTasks::~SelectedJointTasks(){}

bool SelectedJointTasks::_UpdateCommand(const Eigen::VectorXd & _pos_des,
                                      const Eigen::VectorXd & _vel_des,
                                      const Eigen::VectorXd & _acc_des) {

    // Get Actual Values
    Eigen::VectorXd pos_act = Eigen::VectorXd::Zero(dim_task_);
    Eigen::VectorXd vel_act = Eigen::VectorXd::Zero(dim_task_);

    for(int i = 0; i < dim_task_; i++){
        pos_act[i] = robot_->getQ()[joint_indices_[i]];
        vel_act[i] = robot_->getQdot()[joint_indices_[i]];
    }

    // Compute Command
    for (int i(0); i < dim_task_; ++i) {
        op_cmd[i] = _acc_des[i] + kp_[i] * (_pos_des[i] - pos_act[i]) 
                                + kd_[i] * (_vel_des[i] - vel_act[i]);
    }

    return true;
}

bool SelectedJointTasks::_UpdateTaskJacobian(){
    return true;
}

bool SelectedJointTasks::_UpdateTaskJDotQdot(){
    return true;
}
