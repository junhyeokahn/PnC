#pragma once

#include <PnC/WBC/Task.hpp>

class RobotSystem;
class DracoStateProvider;

class CoMxyz : public Task {
 public:
  CoMxyz(RobotSystem*);
  virtual ~CoMxyz();

 protected:
  virtual bool _UpdateCommand(const Eigen::VectorXd& pos_des,
                              const Eigen::VectorXd& vel_des,
                              const Eigen::VectorXd& acc_des);
  
  virtual bool _UpdateCurrent();

  virtual bool _UpdateTaskJacobian();
  virtual bool _UpdateTaskJDotQdot();

  DracoStateProvider* sp_;
};
