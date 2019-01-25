#pragma once

#include <Eigen/Dense>

#include <PnC/WBC/Task.hpp>

class IsolatedPointFootTask : public Task {
    public:
        IsolatedPointFootTask (RobotSystem* robot_,
                      const std::string & linkName_);
        virtual ~IsolatedPointFootTask();

    private:
        virtual bool _UpdateCommand(const Eigen::VectorXd & pos_des,
                                    const Eigen::VectorXd & vel_des,
                                    const Eigen::VectorXd & acc_des);
        virtual bool _UpdateTaskJacobian();
        virtual bool _UpdateTaskJDotQdot();

        // rFoot or lFoot
        std::string link_name_;
};
