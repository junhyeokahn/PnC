#include <PnC/ScorpioPnC/TaskSet/SelectedJointTask.hpp>
#include <Utils/IO/IOUtilities.hpp>

SelectedJointTask::SelectedJointTask(RobotSystem* _robot,
                                     const std::vector<int>& selected_jidx)
    : Task(_robot, selected_jidx.size()) {
    myUtils::pretty_constructor(3, "Selected Joint Task");
    selected_jidx_ = selected_jidx;
    Jt_ = Eigen::MatrixXd::Zero(dim_task_, robot_->getNumDofs());
    JtDotQdot_ = Eigen::VectorXd::Zero(dim_task_);

    for (int i(0); i < selected_jidx_.size(); ++i) {
        Jt_(i, selected_jidx_[i]) = 1.;
    }
}

SelectedJointTask::~SelectedJointTask() {}

bool SelectedJointTask::_UpdateCommand(const Eigen::VectorXd& _pos_des,
                                       const Eigen::VectorXd& _vel_des,
                                       const Eigen::VectorXd& _acc_des) {
    // pos_err, vel_des, acc_des
    for (int i(0); i < selected_jidx_.size(); ++i) {
        pos_err[i] = _pos_des[i] - (robot_->getQ())[selected_jidx_[i]];
        vel_des[i] = _vel_des[i];
        acc_des[i] = _acc_des[i];
        op_cmd[i] = _acc_des[i] + kp_[i]*pos_err[i] +
            kd_[i]*(vel_des[i] - (robot_->getQdot())[selected_jidx_[i]]);
    }
    //std::cout << "--------------------------------------------" << std::endl;
    //myUtils::pretty_print(_pos_des, std::cout, "pos des");
    //myUtils::pretty_print(_vel_des, std::cout, "vel des");
    //myUtils::pretty_print(_acc_des, std::cout, "acc des");
    //myUtils::pretty_print(kp_, std::cout, "kp");
    //myUtils::pretty_print(kd_, std::cout, "kd");
    //myUtils::pretty_print(pos_err, std::cout, "pos err");
    //myUtils::pretty_print(op_cmd, std::cout, "op cmd");
    return true;
}

bool SelectedJointTask::_UpdateTaskJacobian() {
    for (int i(0); i < selected_jidx_.size(); ++i) {
        Jt_(i, selected_jidx_[i]) = 1.;
    }
    return true;
}

bool SelectedJointTask::_UpdateTaskJDotQdot() {
    JtDotQdot_ = Eigen::VectorXd::Zero(dim_task_);
    return true;
}
