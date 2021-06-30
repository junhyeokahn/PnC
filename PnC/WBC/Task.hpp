#pragma once

#include <stdio.h>
#include <string>

#include <PnC/RobotSystem/RobotSystem.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/PseudoInverse.hpp>

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
    myUtils::pretty_print(pos_des, std::cout, "pos des");
    myUtils::pretty_print(pos_err, std::cout, "pos err");
    myUtils::pretty_print(vel_des, std::cout, "vel des");
    myUtils::pretty_print(acc_des, std::cout, "acc des");
    myUtils::pretty_print(op_cmd, std::cout, "xddot");
  }

protected:
  RobotSystem *robot_;
};
