#pragma once

#include "pnc/robot_system/robot_system.hpp"

class InternalConstraint {
public:
  InternalConstraint(RobotSystem *_robot, const int &_dim) {
    robot_ = _robot;
    dim_ = _dim;

    jacobian = Eigen::MatrixXd::Zero(dim_, robot_->n_q_dot);
    jacobian_dot_q_dot = Eigen::VectorXd::Zero(dim_);
  };
  virtual ~InternalConstraint(){};

  void update_internal_constraint() { _update_jacobian(); };

  Eigen::MatrixXd jacobian;
  Eigen::VectorXd jacobian_dot_q_dot;

protected:
  /* data */
  RobotSystem *robot_;
  int dim_;

  virtual void _update_jacobian() = 0;
};
