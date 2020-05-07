#pragma once

#include <Eigen/Dense>

#include <PnC/WBC/Task.hpp>

class AngularMomentumTask : public Task {
   public:
    AngularMomentumTask(RobotSystem* robot_, const double dt_eps);
    virtual ~AngularMomentumTask();

   private:
    /* Update op_cmd, pos_err, vel_des, acc_des
     *
     * vel_des_ = [w_x, w_y, w_z] // angular momentum
     * acc_des_ = [a_x, a_y, a_z] // angular momentum rate
     *
     */
    virtual bool _UpdateCommand(const Eigen::VectorXd& pos_des,
                                const Eigen::VectorXd& vel_des,
                                const Eigen::VectorXd& acc_des);
    virtual bool _UpdateTaskJacobian();
    virtual bool _UpdateTaskJDotQdot();

    Eigen::MatrixXd Ag_cur_; // Current centroidal inertia matrix
    Eigen::MatrixXd Ag_prev_; // Previous centroidal inertia matrix
    Eigen::MatrixXd Agdot_; // Current Estimate for centroidal inertia matrix
    double dt_eps_; // time interval to use for approximating Agdot_
    bool first_pass_; // first time computing Jdotqdot

};
