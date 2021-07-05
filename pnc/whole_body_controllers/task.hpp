#pragma once

#include <stdio.h>
#include <string>

#include "pnc/robot_system/robot_system.hpp"
#include "utils/util.hpp"

/*
 * WBC Task
 * --------
 * Usage:
 *     update_desired --> update_jacobian --> update_cmd
 */
class Task {
public:
  Task(
      RobotSystem *_robot, const int &_dim,
      const std::vector<std::string> _target_ids = std::vector<std::string>()) {
    robot_ = _robot;
    dim = _dim;
    target_ids = _target_ids;

    w_hierarchy = 1.0;

    kp = Eigen::VectorXd::Zero(dim);
    kd = Eigen::VectorXd::Zero(dim);

    jacobian = Eigen::MatrixXd::Zero(dim, robot_->n_q_dot);
    jacobian_dot_q_dot = Eigen::VectorXd::Zero(dim);

    op_cmd = Eigen::VectorXd::Zero(dim);
    pos_err = Eigen::VectorXd::Zero(dim);

    pos_des = Eigen::VectorXd::Zero(dim);
    pos = Eigen::VectorXd::Zero(dim);
    vel_des = Eigen::VectorXd::Zero(dim);
    vel = Eigen::VectorXd::Zero(dim);
    acc_des = Eigen::VectorXd::Zero(dim);
  };
  virtual ~Task(){};

  int dim;

  double w_hierarchy;
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;

  Eigen::MatrixXd jacobian;
  Eigen::VectorXd jacobian_dot_q_dot;

  Eigen::VectorXd op_cmd;
  Eigen::VectorXd pos_err;

  Eigen::VectorXd pos_des;
  Eigen::VectorXd pos;
  Eigen::VectorXd vel_des;
  Eigen::VectorXd vel;
  Eigen::VectorXd acc_des;

  std::vector<std::string> target_ids;

  /*
   *  Update pos_des, vel_des, acc_des which will be used later to compute
   *  op_cmd

   *  Parameters
   *  ----------
   *  pos_des (np.array):
   *      For orientation task, the size of numpy array is 4, and it should
   *      be represented in scalar-last quaternion
   *  vel_des (np.array):
   *      Velocity desired
   *  acc_des (np.array):
   *      Acceleration desired
   */
  void update_desired(const Eigen::VectorXd &_pos_des,
                      const Eigen::VectorXd &_vel_des,
                      const Eigen::VectorXd &_acc_des) {
    pos_des = _pos_des;
    vel_des = _vel_des;
    acc_des = _acc_des;
  }

  /*
   * Update op_cmd, pos_err given updated pos_des, vel_des, acc_des
   */
  virtual void update_cmd() = 0;

  /*
   * Update jacobian and jacobian_dot_q_dot
   */
  virtual void update_jacobian() = 0;

  void Debug() {
    std::cout << "pos des" << std::endl;
    std::cout << pos_des << std::endl;
    std::cout << "pos err" << std::endl;
    std::cout << pos_err << std::endl;
    std::cout << "vel des" << std::endl;
    std::cout << vel_des << std::endl;
    std::cout << "acc des" << std::endl;
    std::cout << acc_des << std::endl;
    std::cout << "xddot" << std::endl;
    std::cout << op_cmd << std::endl;
  }

  void CopyData(Eigen::VectorXd &_pos_des, Eigen::VectorXd &_vel_des,
                Eigen::VectorXd &_acc_des, Eigen::VectorXd &_pos,
                Eigen::VectorXd &_vel) {
    _pos_des = pos_des;
    _vel_des = vel_des;
    _acc_des = acc_des;
    _pos = pos;
    _vel = vel;
  }

protected:
  RobotSystem *robot_;
};
