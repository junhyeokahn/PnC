#pragma once

#include <Eigen/Dense>

#include <PnC/WBC/Task.hpp>

class PitchFootTask : public Task {
    public:
        PitchFootTask (RobotSystem* robot_,
                      const std::string & linkName_);
        virtual ~PitchFootTask();

    private:
        /* Update op_cmd, pos_err, vel_des, acc_des
         *
         * pos_des_ = [quat_w, quat_x, quat_y, quat_z, x, y, z]
         * vel_des_ = [w_x, w_y, w_z, xdot, ydot, zdot]
         * acc_des_ = [a_x, a_y, a_z, xddot, yddot, zddot]
         *
         */
        virtual bool _UpdateCommand(const Eigen::VectorXd & pos_des,
                                    const Eigen::VectorXd & vel_des,
                                    const Eigen::VectorXd & acc_des);
        virtual bool _UpdateTaskJacobian();
        virtual bool _UpdateTaskJDotQdot();

        // rFoot or lFoot
        std::string link_name_;
};
