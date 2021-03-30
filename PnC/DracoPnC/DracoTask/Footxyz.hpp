#pragma once

#include <Eigen/Dense>

#include <PnC/WBC/Task.hpp>

class Footxyz : public Task {
 public:
  Footxyz(RobotSystem* robot_, int _link_idx_);
  virtual ~Footxyz();

 private:
  virtual bool _UpdateCommand(const Eigen::VectorXd& pos_des,
                              const Eigen::VectorXd& vel_des,
                              const Eigen::VectorXd& acc_des);
  virtual bool _UpdateTaskJacobian();
  virtual bool _UpdateTaskJDotQdot();
  virtual bool _UpdateCurrent();   
  int link_idx_;
  int getLinkID() { return link_idx_; }
};
