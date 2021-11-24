#pragma once

#include <stdio.h>
#include <string>

#include "pnc/robot_system/robot_system.hpp"
#include "utils/util.hpp"

/// class Task
class Task {
public:
  /// \{ \name Constructor and Destructor
  Task(
      RobotSystem *_robot, const int &_dim,
      const std::vector<std::string> _target_ids = std::vector<std::string>()) {
    robot_ = _robot;
    dim = _dim;
    target_ids = _target_ids;

    w_hierarchy = 1.0;

    kp = Eigen::VectorXd::Zero(dim);
    kd = Eigen::VectorXd::Zero(dim);
    ki = Eigen::VectorXd::Zero(dim);

    jacobian = Eigen::MatrixXd::Zero(dim, robot_->n_q_dot);
    jacobian_dot_q_dot = Eigen::VectorXd::Zero(dim);

    op_cmd = Eigen::VectorXd::Zero(dim);
    pos_err = Eigen::VectorXd::Zero(dim);
    local_pos_err = Eigen::VectorXd::Zero(dim);
    local_vel_err = Eigen::VectorXd::Zero(dim);

    pos_des = Eigen::VectorXd::Zero(dim);
    pos = Eigen::VectorXd::Zero(dim);
    vel_des = Eigen::VectorXd::Zero(dim);
    vel = Eigen::VectorXd::Zero(dim);
    acc_des = Eigen::VectorXd::Zero(dim);
    rot_world_local_ = Eigen::MatrixXd::Identity(dim, dim);
  };

  virtual ~Task(){};
  /// \}

  /// Task dimension.
  int dim;

  /// Task hierarchy weight.
  double w_hierarchy;

  /// Task P gain.
  Eigen::VectorXd kp;

  /// Task D gain.
  Eigen::VectorXd kd;

  /// Task I gain.
  Eigen::VectorXd ki;

  /// Task jacobian.
  Eigen::MatrixXd jacobian;

  /// Task jacobian times qdot.
  Eigen::VectorXd jacobian_dot_q_dot;

  /// Accelration command.
  Eigen::VectorXd op_cmd;

  /// Position error.
  Eigen::VectorXd pos_err;

  /// Local frame position error.
  Eigen::VectorXd local_pos_err;

  /// Local frame velocity error.
  Eigen::VectorXd local_vel_err;

  /// Desired Position.
  Eigen::VectorXd pos_des;

  /// Measured position.
  Eigen::VectorXd pos;

  /// Desired velocity.
  Eigen::VectorXd vel_des;

  /// Measured velocity.
  Eigen::VectorXd vel;

  /// Desired acceleration.
  Eigen::VectorXd acc_des;

  /// Task target ids.
  std::vector<std::string> target_ids;

  /// Update pos_des, vel_des, acc_des which will be used later to compute
  /// op_cmd
  /// \note For orientation task, _pos_des has 4 dimensional vector that
  /// represents scalar first quaternion.
  /// \note For the angular momentum task, _pos_des is ignored.
  void update_desired(const Eigen::VectorXd &_pos_des,
                      const Eigen::VectorXd &_vel_des,
                      const Eigen::VectorXd &_acc_des) {
    pos_des = _pos_des;
    vel_des = _vel_des;
    acc_des = _acc_des;
  }

  /// Update task command.
  virtual void update_cmd() = 0;

  /// Update task jacobian.
  virtual void update_jacobian() = 0;

  /// Print outs for debugging purpose.
  void Debug() {
    std::cout << "pos des" << std::endl;
    std::cout << pos_des << std::endl;
    std::cout << "pos err" << std::endl;
    std::cout << pos_err << std::endl;
    std::cout << "local pos err" << std::endl;
    std::cout << local_pos_err << std::endl;
    std::cout << "vel des" << std::endl;
    std::cout << vel_des << std::endl;
    std::cout << "acc des" << std::endl;
    std::cout << acc_des << std::endl;
    std::cout << "xddot" << std::endl;
    std::cout << op_cmd << std::endl;
  }

  /// Copies task commands.
  void CopyData(Eigen::VectorXd &_pos_des, Eigen::VectorXd &_vel_des,
                Eigen::VectorXd &_acc_des, Eigen::VectorXd &_pos,
                Eigen::VectorXd &_vel) {
    _pos_des = pos_des;
    _vel_des = vel_des;
    _acc_des = acc_des;
    _pos = pos;
    _vel = vel;
  }

  /// Copies task commands.
  void CopyData(Eigen::VectorXd &_pos_des, Eigen::VectorXd &_vel_des,
                Eigen::VectorXd &_acc_des, Eigen::VectorXd &_pos,
                Eigen::VectorXd &_vel, Eigen::VectorXd &_local_pos_error,
                Eigen::VectorXd &_local_vel_error) {
    _pos_des = pos_des;
    _vel_des = vel_des;
    _acc_des = acc_des;
    _pos = pos;
    _vel = vel;
    _local_pos_error = local_pos_err;
    _local_vel_error = local_vel_err;
  }

protected:
  RobotSystem *robot_;
  Eigen::MatrixXd rot_world_local_;
};
