#pragma once

#include <PnC/WBC/Task.hpp>

class RobotSystem;

class SelectedJointTask : public Task {
 public:
  SelectedJointTask(RobotSystem* robot, const std::vector<int>& joint_indices);
  virtual ~SelectedJointTask();

 protected:
  /* Update pos_err, vel_des, acc_des
   *
   * pos_des_ = [q_i, q_i+1, ..., q_i+n]
   * vel_des_ = [dq_i, dq_i+1, ..., dq_i+n]
   * acc_des_ = [ddq_i, dq_i+1, ..., ddq_i+n]
   */
  virtual bool _UpdateCommand(const Eigen::VectorXd& pos_des,
                              const Eigen::VectorXd& vel_des,
                              const Eigen::VectorXd& acc_des);
  virtual bool _UpdateTaskJacobian();
  virtual bool _UpdateTaskJDotQdot();
  virtual bool _UpdateCurrent(); 

  std::vector<int> joint_indices_;
};
