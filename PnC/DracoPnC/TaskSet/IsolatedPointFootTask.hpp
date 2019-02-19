#pragma once

#include <Eigen/Dense>

#include <PnC/WBC/Task.hpp>

class IsolatedPointFootTask : public Task {
   public:
    IsolatedPointFootTask(RobotSystem* robot_, int _link_idx);
    virtual ~IsolatedPointFootTask();

   private:
    virtual bool _UpdateCommand(const Eigen::VectorXd& pos_des,
                                const Eigen::VectorXd& vel_des,
                                const Eigen::VectorXd& acc_des);
    virtual bool _UpdateTaskJacobian();
    virtual bool _UpdateTaskJDotQdot();

    int link_idx_;
};
