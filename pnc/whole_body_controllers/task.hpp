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

    w_hierarchy = Eigen::VectorXd::Ones(dim);

    kp = Eigen::VectorXd::Zero(dim);
    kd = Eigen::VectorXd::Zero(dim);
    ki = Eigen::VectorXd::Zero(dim);

    jacobian = Eigen::MatrixXd::Zero(dim, robot_->n_q_dot);
    jacobian_dot_q_dot = Eigen::VectorXd::Zero(dim);

    op_cmd = Eigen::VectorXd::Zero(dim);

    pos_err = Eigen::VectorXd::Zero(dim);
    local_pos_err = Eigen::VectorXd::Zero(dim);

    vel_err = Eigen::VectorXd::Zero(dim);
    local_vel_err = Eigen::VectorXd::Zero(dim);

    pos_des = Eigen::VectorXd::Zero(dim);
    local_pos_des = Eigen::VectorXd::Zero(dim);

    pos = Eigen::VectorXd::Zero(dim);
    local_pos = Eigen::VectorXd::Zero(dim);

    vel_des = Eigen::VectorXd::Zero(dim);
    local_vel_des = Eigen::VectorXd::Zero(dim);

    vel = Eigen::VectorXd::Zero(dim);
    local_vel = Eigen::VectorXd::Zero(dim);

    acc_des = Eigen::VectorXd::Zero(dim);
    local_acc_des = Eigen::VectorXd::Zero(dim);
  };

  virtual ~Task(){};
  /// \}

  /// Task dimension.
  int dim;

  /// Task hierarchy weight.
  Eigen::VectorXd w_hierarchy;

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

  /// Position error.
  Eigen::VectorXd vel_err;

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

  /// Desired Position.
  Eigen::VectorXd local_pos_des;

  /// Measured position.
  Eigen::VectorXd local_pos;

  /// Desired velocity.
  Eigen::VectorXd local_vel_des;

  /// Measured velocity.
  Eigen::VectorXd local_vel;

  /// Desired acceleration.
  Eigen::VectorXd local_acc_des;

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
  virtual void
  update_cmd(Eigen::Matrix3d rot_world_local = Eigen::Matrix3d::Identity()) = 0;

  /// Update task jacobian.
  virtual void update_jacobian() = 0;

  void ignore_jacobian_row(std::vector<int> idx) {
    for (int col_id = 0; col_id < idx.size(); ++col_id) {
      for (int row_id = 0; row_id < jacobian.rows(); ++row_id) {
        jacobian(row_id, col_id) = 0.;
      }
    }
  }

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
                Eigen::VectorXd &_vel, Eigen::VectorXd &_local_pos_des,
                Eigen::VectorXd &_local_vel_des,
                Eigen::VectorXd &_local_acc_des, Eigen::VectorXd &_local_pos,
                Eigen::VectorXd &_local_vel) {
    _pos_des = pos_des;
    _vel_des = vel_des;
    _acc_des = acc_des;
    _pos = pos;
    _vel = vel;

    _local_pos_des = local_pos_des;
    _local_vel_des = local_vel_des;
    _local_acc_des = local_acc_des;
    _local_pos = local_pos;
    _local_vel = local_vel;
  }

protected:
  RobotSystem *robot_;
};
