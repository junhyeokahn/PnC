#pragma once

#include <PnC/WBC/Task.hpp>

class RobotSystem;
class LaikagoStateProvider;

class DCMTask : public Task {
 public:
  DCMTask(RobotSystem*);
  virtual ~DCMTask();

 protected:
  /* Update pos_err, vel_des, acc_des
   *
   * pos_des_ = [dcm_x, dcm_y, com_z]
   * vel_des_ = [dcm_dot_x, dcm_dot_y, com_vel_z]
   * acc_des_ = [a_x, a_y, a_z, x_ddot, y_ddot, z_ddot]
   */
  virtual bool _UpdateCommand(const Eigen::VectorXd& pos_des,
                              const Eigen::VectorXd& vel_des,
                              const Eigen::VectorXd& acc_des);
  virtual bool _UpdateTaskJacobian();
  virtual bool _UpdateTaskJDotQdot();

  LaikagoStateProvider* sp_;
};
