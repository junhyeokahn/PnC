#pragma once

#include <PnC/WBC/Task.hpp>

class RobotSystem;

class SelectedJointTask2: public Task{
public:
  SelectedJointTask2(RobotSystem* _robot, const std::vector<int> & selected_jidx);
  virtual ~SelectedJointTask2();

protected:
  virtual bool _UpdateCommand(const Eigen::VectorXd & pos_des,
                              const Eigen::VectorXd & vel_des,
                              const Eigen::VectorXd & acc_des);
  virtual bool _UpdateTaskJacobian();
  virtual bool _UpdateTaskJDotQdot();

  std::vector<int> selected_jidx_;
};
