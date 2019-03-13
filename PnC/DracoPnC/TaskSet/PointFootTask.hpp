#pragma once

#include <Eigen/Dense>

#include <PnC/WBC/Task.hpp>

class PointFootTask : public Task {
   public:
    PointFootTask(RobotSystem* robot_, int _link_idx);
    virtual ~PointFootTask();

   private:
    virtual bool _UpdateCommand(const Eigen::VectorXd& pos_des,
                                const Eigen::VectorXd& vel_des,
                                const Eigen::VectorXd& acc_des);
    virtual bool _UpdateTaskJacobian();
    virtual bool _UpdateTaskJDotQdot();

    int link_idx_;
};
