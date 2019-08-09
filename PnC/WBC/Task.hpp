#pragma once

#include <stdio.h>

#include <PnC/RobotSystem/RobotSystem.hpp>

class Task {
   public:
    Task(RobotSystem* _robot, const int& _dim) {
        robot_ = _robot;
        b_set_task_ = false;
        dim_task_ = _dim;
        kp_ = Eigen::VectorXd::Zero(_dim);
        kd_ = Eigen::VectorXd::Zero(_dim);
        JtDotQdot_ = Eigen::VectorXd::Zero(_dim);
        Jt_ = Eigen::MatrixXd::Zero(_dim, robot_->getNumDofs());

        op_cmd = Eigen::VectorXd::Zero(_dim);

        pos_err = Eigen::VectorXd::Zero(_dim);
        vel_des = Eigen::VectorXd::Zero(_dim);
        acc_des = Eigen::VectorXd::Zero(_dim);
    }
    virtual ~Task() {}

    void getCommand(Eigen::VectorXd& _op_cmd) { _op_cmd = op_cmd; }
    void getTaskJacobian(Eigen::MatrixXd& Jt) { Jt = Jt_; }
    void getTaskJacobianDotQdot(Eigen::VectorXd& JtDotQdot) {
        JtDotQdot = JtDotQdot_;
    }
    void setGain(const Eigen::VectorXd& _kp, const Eigen::VectorXd& _kd) {
        kp_ = _kp;
        kd_ = _kd;
    }
    void PrintInfos() {
        myUtils::pretty_print(pos_err, std::cout, "pos err");
        myUtils::pretty_print(Jt_, std::cout, "task jacobian");
    }

    bool updateTask(const Eigen::VectorXd& pos_des,
                    const Eigen::VectorXd& vel_des,
                    const Eigen::VectorXd& acc_des) {
        _UpdateTaskJacobian();
        _UpdateTaskJDotQdot();
        _UpdateCommand(pos_des, vel_des, acc_des);
        b_set_task_ = true;
        return true;
    }

    bool isTaskSet() { return b_set_task_; }
    int getDim() { return dim_task_; }
    void unsetTask() { b_set_task_ = false; }

    // For Dyn WBC
    Eigen::VectorXd op_cmd;
    // For Kin WBC
    Eigen::VectorXd pos_err;
    Eigen::VectorXd vel_des;
    Eigen::VectorXd acc_des;

   protected:
    virtual bool _UpdateCommand(const Eigen::VectorXd& pos_des,
                                const Eigen::VectorXd& vel_des,
                                const Eigen::VectorXd& acc_des) = 0;
    virtual bool _UpdateTaskJacobian() = 0;
    virtual bool _UpdateTaskJDotQdot() = 0;

    RobotSystem* robot_;
    bool b_set_task_;
    int dim_task_;
    Eigen::VectorXd kp_;
    Eigen::VectorXd kd_;

    Eigen::VectorXd JtDotQdot_;
    Eigen::MatrixXd Jt_;
};
