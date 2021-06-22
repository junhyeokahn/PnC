#pragma once

class InternalConstraint {
public:
  InternalConstraint(RobotSystem *_robot, const int &_dim) {
    robot_ = _robot;
    dim_ = _dim;
  };
  virtual ~InternalConstraint(){};

  void update_internal_constraint() { _update_jacobian(); };

  Eigen::MatrixXd jacobian;
  Eigen::VectorXd jacobian_dot_q_dot;

private:
  /* data */
  RobotSystem *robot_;
  int dim_;

  virtual void _update_jacobian() = 0;
};
