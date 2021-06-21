#pragma once

#include <stdio.h>
#include <string>

#include <PnC/RobotSystem/RobotSystem.hpp>

/*
 * WBC Task
 * --------
 * Usage:
 *     update_desired --> update_jacobian --> update_cmd
 */
class Task {
public:
  Task(RobotSystem *_robot, const int &_dim,
       const std::vector<std::string> _target_ids) {
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

    pos_des_ = Eigen::VectorXd::Zero(dim);
    vel_des_ = Eigen::VectorXd::Zero(dim);
    acc_des_ = Eigen::VectorXd::Zero(dim);
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
  void update_desired(const Eigen::VectorXd &pos_des,
                      const Eigen::VectorXd &vel_des,
                      const Eigen::VectorXd &acc_des) {
    pos_des_ = pos_des;
    vel_des_ = vel_des;
    acc_des_ = acc_des;
  }

  /*
   * Update op_cmd, pos_err given updated pos_des_, vel_des_, acc_des_
   */
  virtual void update_cmd() = 0;

  /*
   * Update jacobian and jacobian_dot_q_dot
   */
  virtual void update_jacobian() = 0;

protected:
  RobotSystem *robot_;

  Eigen::VectorXd pos_des_;
  Eigen::VectorXd vel_des_;
  Eigen::VectorXd acc_des_;
};
