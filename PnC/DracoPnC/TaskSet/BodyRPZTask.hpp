#pragma once

#include <PnC/WBC/Task.hpp>

class RobotSystem;

class BodyRPZTask: public Task{
    public:
        BodyRPZTask(RobotSystem*);
        virtual ~BodyRPZTask();

    protected:
        /* Update pos_err, vel_des, acc_des
         *
         * pos_des_ = [quat_x, quat_y, quat_z, quat_w, x, y, z]
         * vel_des_ = [r_dot, p_dot, y_dot, x_dot, y_dot, z_dot]
         * acc_des_ = [r_ddot, p_ddot, y_ddot, x_ddot, y_ddot, z_ddot]
         */
        virtual bool _UpdateCommand(const Eigen::VectorXd & pos_des,
                const Eigen::VectorXd & vel_des,
                const Eigen::VectorXd & acc_des);
        virtual bool _UpdateTaskJacobian();
        virtual bool _UpdateTaskJDotQdot();

        DracoStateProvider* sp_;
};
