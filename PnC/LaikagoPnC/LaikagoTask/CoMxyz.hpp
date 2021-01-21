#pragma once

#include <PnC/WBC/Task.hpp>

class RobotSystem;
class LaikagoStateProvider;

class CoMxyz : public Task {
 public:
  CoMxyz(RobotSystem*);
  virtual ~CoMxyz();

 protected:
  virtual bool _UpdateCommand(const Eigen::VectorXd& pos_des,
                              const Eigen::VectorXd& vel_des,
                              const Eigen::VectorXd& acc_des);
  virtual bool _UpdateTaskJacobian();
  virtual bool _UpdateTaskJDotQdot();

  LaikagoStateProvider* sp_;
};
