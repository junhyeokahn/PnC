#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <stdio.h>

#include "pnc/robot_system/robot_system.hpp"
#include "utils/util.hpp"

/// class Contact
class Contact {
public:
  /// \{ \name Constructor and Destructor
  Contact(RobotSystem *_robot, const int _dim, const std::string _target_id,
          const double _mu) {
    robot_ = _robot;

    dim = _dim;
    target_id = _target_id;
    mu_ = _mu;
    rf_z_max = 1000.;

    jacobian = Eigen::MatrixXd::Zero(dim, robot_->n_q_dot);
    jacobian_dot_q_dot = Eigen::VectorXd::Zero(dim);
  }

  virtual ~Contact() {}
  /// \}

  /// Contact dimension
  int dim;

  /// Contact frame
  std::string target_id;

  /// Maximum normal force
  double rf_z_max;

  /// Contact jacobian
  Eigen::MatrixXd jacobian;

  /// Contact jacobian times qdot
  Eigen::MatrixXd jacobian_dot_q_dot;

  /// Contact constraint matrix
  Eigen::MatrixXd cone_constraint_mat;

  /// Contact constraint vector
  Eigen::VectorXd cone_constraint_vec;

  /// Update contact jacobian and contact constraint.
  void update_contact() {
    this->_update_jacobian();
    this->_update_cone_constraint();
  };

protected:
  RobotSystem *robot_;

  virtual void _update_jacobian() = 0;
  virtual void _update_cone_constraint() = 0;

  double mu_;
};
