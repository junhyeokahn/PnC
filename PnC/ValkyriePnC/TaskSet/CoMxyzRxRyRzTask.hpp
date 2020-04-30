#pragma once

#include <PnC/WBC/Task.hpp>

class RobotSystem;

class CoMxyzRxRyRzTask : public Task {
   public:
    CoMxyzRxRyRzTask(RobotSystem*);
    virtual ~CoMxyzRxRyRzTask();

   protected:
    /* Update pos_err, vel_des, acc_des
     *
     * pos_des_ = [quat_w, quat_x, quat_y, quat_z, x, y, z]
     * vel_des_ = [w_x, w_y, y_z, x_dot, y_dot, z_dot]
     * acc_des_ = [a_x, a_y, a_z, x_ddot, y_ddot, z_ddot]
     */
    virtual bool _UpdateCommand(const Eigen::VectorXd& pos_des,
                                const Eigen::VectorXd& vel_des,
                                const Eigen::VectorXd& acc_des);
    virtual bool _UpdateTaskJacobian();
    virtual bool _UpdateTaskJDotQdot();
};
