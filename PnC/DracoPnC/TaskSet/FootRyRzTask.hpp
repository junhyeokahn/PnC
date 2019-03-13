#pragma once

#include <Eigen/Dense>

#include <PnC/WBC/Task.hpp>

class FootRyRzTask : public Task {
   public:
    FootRyRzTask(RobotSystem* robot_, int _link_idx_);
    virtual ~FootRyRzTask();

   private:
    /* Update op_cmd, pos_err, vel_des, acc_des
     *
     * pos_des_ = [quat_w, quat_x, quat_y, quat_z]
     * vel_des_ = [w_x, w_y, w_z]
     * acc_des_ = [a_x, a_y, a_z]
     *
     */
    virtual bool _UpdateCommand(const Eigen::VectorXd& pos_des,
                                const Eigen::VectorXd& vel_des,
                                const Eigen::VectorXd& acc_des);
    virtual bool _UpdateTaskJacobian();
    virtual bool _UpdateTaskJDotQdot();

    int link_idx_;
};
