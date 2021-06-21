#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <stdio.h>

#include <PnC/RobotSystem/RobotSystem.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

/*
 * WBC Contact
 * -----------
 * Usage:
 *     update_contact
 */
class Contact {
public:
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

  int dim;
  std::string target_id;
  double rf_z_max;

  Eigen::MatrixXd jacobian;
  Eigen::MatrixXd jacobian_dot_q_dot;

  Eigen::MatrixXd cone_constraint_mat;
  Eigen::VectorXd cone_constraint_vec;

  void update_contact() {
    this->_update_jacobian();
    this->_update_cone_constraint();
  };

protected:
  RobotSystem *robot_;

  virtual void _update_jacobian() = 0;
  virtual void _update_cone_constraint() = 0;

  Eigen::MatrixXd _get_u(double _x, double _y, double _mu);

  double mu_;
};
