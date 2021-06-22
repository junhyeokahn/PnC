#pragma once

#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>

class RobotSystem {
public:
  RobotSystem(const bool _b_fixed_base, const bool _b_print_info)
      : b_fixed_base(_b_fixed_base), b_print_info(_b_print_info){};

  virtual ~RobotSystem(){};

  bool b_fixed_base;
  bool b_print_info;

  int n_floating;
  int n_q;
  int n_q_dot;
  int n_a;

  double total_mass;

  Eigen::Matrix<double, Eigen::Dynamic, 2> joint_pos_limit;
  Eigen::Matrix<double, Eigen::Dynamic, 2> joint_vel_limit;
  Eigen::Matrix<double, Eigen::Dynamic, 2> joint_trq_limit;

  Eigen::VectorXd joint_positions;
  Eigen::VectorXd joint_velocities;

  Eigen::Matrix<double, 6, 6> Ig;
  Eigen::Matrix<double, 6, Eigen::Dynamic> Ag;
  Eigen::Matrix<double, 6, 1> hg;

  /*
   * Get joint index in generalized coordinate
   *
   * Parameters
   * ----------
   *  joint_name (str)
   *
   *  Returns
   *  -------
   *  joint_idx (int)
   */
  virtual int get_q_idx(const std::string joint_name) = 0;

  /*
   * Get joint velocity index in generalized coordinate
   *
   * Parameters
   * ----------
   *  joint_name (str)
   *
   *  Returns
   *  -------
   *  joint_idx (int)
   */
  virtual int get_q_dot_idx(const std::string joint_name) = 0;

  /*
   * Get joint index in generalized coordinate
   *
   * Parameters
   * ----------
   * joint_name (str)
   *
   * Returns
   * -------
   * joint_idx (int)
   */
  virtual int get_joint_idx(const std::string joint_name) = 0;

  /*
   * Create command map
   *
   * Parameters
   * ----------
   * cmd_vec (Eigen::Vector)
   *
   * Returns
   * -------
   * cmd_map (std::map)
   */
  virtual std::map<std::string, double>
  create_cmd_map(const Eigen::VectorXd cmd_vec) = 0;

  /*
   * Update generalized coordinate
   *
   * Parameters
   * ----------
   * base_pos (np.array): Root pos, None if the robot is fixed in the world
   * base_quat (np.array): Root quat
   * base_lin_vel (np.array): Root linear velocity
   * base_ang_vel (np.array): Root angular velocity
   * joint_pos (OrderedDict): Actuator pos
   * joint_vel (OrderedDict): Actuator vel
   * b_cent (Bool): Whether updating centroidal frame or not
   */
  virtual void update_system(const Eigen::Vector3d base_com_pos,
                             const Eigen::Quaternion<double> base_com_quat,
                             const Eigen::Vector3d base_com_lin_vel,
                             const Eigen::Vector3d base_com_ang_vel,
                             const Eigen::Vector3d base_joint_pos,
                             const Eigen::Quaternion<double> base_joint_quat,
                             const Eigen::Vector3d base_joint_lin_vel,
                             const Eigen::Vector3d base_joint_ang_vel,
                             const std::map<std::string, double> joint_pos,
                             const std::map<std::string, double> joint_vel,
                             const bool b_cent = false) = 0;

  /*
   * Returns
   * -------
   * q (Eigen::Vector): positions in generalized coordinate
   */
  virtual Eigen::VectorXd get_q() = 0;

  /*
   * Returns
   * -------
   * qdot (Eigen::Vector): velocities in generalized coordinate
   */
  virtual Eigen::VectorXd get_q_dot() = 0;

  /*
   * Returns
   * -------
   * A (Eigen::Matrix): mass matrix in generalized coordinate
   */
  virtual Eigen::MatrixXd get_mass_matrix() = 0;

  /*
   * Returns
   * -------
   * g (Eigen::Vector): Gravity forces in generalized coordinate
   */
  virtual Eigen::VectorXd get_gravity() = 0;

  /*
   * Returns
   * -------
   * c (Eigen::Vector): Coriolis forces in generalized coordinate
   */
  virtual Eigen::VectorXd get_coriolis() = 0;

  /*
   * Returns
   * -------
   * com_pos (Eigen::Vector): COM position
   */
  virtual Eigen::Vector3d get_com_pos() = 0;

  /*
   * Returns
   * -------
   * com_lin_vel (Eigen::Vector): COM velocity
   */
  virtual Eigen::Vector3d get_com_lin_vel() = 0;

  /*
   * Returns
   * -------
   * com_lin_jacobian (Eigen::Vector): COM jacobian
   */
  virtual Eigen::Matrix<double, 3, Eigen::Dynamic> get_com_lin_jacobian() = 0;

  /*
   * Returns
   * -------
   * com_lin_jacobian_dot (Eigen::Vector): COM jacobian dot
   */
  virtual Eigen::Matrix<double, 3, Eigen::Dynamic>
  get_com_lin_jacobian_dot() = 0;

  /*
   * Parameters
   * ----------
   * link_id (str):
   *     Link ID
   *
   * Returns
   * -------
   * link_iso (Eigen::Isometry): Link SE(3)
   */
  virtual Eigen::Isometry3d get_link_iso(const std::string link_id) = 0;

  /*
   * Parameters
   * ----------
   * link_id (str):
   *     Link ID
   *
   * Returns
   * -------
   * link_vel (Eigen::Vector): Link COM Screw described in World Frame
   */
  virtual Eigen::Matrix<double, 6, 1>
  get_link_vel(const std::string link_id) = 0;

  /*
   * Parameters
   * ----------
   * link_id (str):
   *     Link ID
   *
   * Returns
   * -------
   * jac (Eigen::Matrix):
   *     Link COM Jacobian in World Frame
   */
  virtual Eigen::Matrix<double, 6, Eigen::Dynamic>
  get_link_jacobian(const std::string link_id) = 0;

  /*
   * Parameters
   * ----------
   * link_id (str):
   *     Link ID
   *
   * Returns
   * -------
   * jac_dot_times_qdot (Eigen::Matrix):
   *     Link COM Jacobian_dot times q_dot
   */
  virtual Eigen::Matrix<double, 6, 1>
  get_link_jacobian_dot_times_qdot(const std::string link_id) = 0;

private:
  /*
   * Update Ig, Ag, hg:
   *    hg = Ig * centroid_velocity = Ag * qdot
   * Note that all quantities are represented in the world frame
   */
  virtual void _update_centroidal_quantities() = 0;

  /*
   * Configure following properties:
   *     n_floating (int):
   *         Number of floating joints
   *     n_q (int):
   *         Size of joint positions in generalized coordinate
   *     n_q_dot (int):
   *         Size of joint velocities in generalized coordinate
   *     n_a (int):
   *         Size of actuation in generalized coordinate
   *     total_mass (double):
   *         Total mass of the robot
   *     joint_pos_limit (Eigen::Matrix):
   *         Joint position limits. Size of (n_a, 2)
   *     joint_vel_limit (Eigen::Matrix):
   *         Joint velocity limits. Size of (n_a, 2)
   *     joint_trq_limit (Eigen::Matrix):
   *         Joint torque limits. Size of (n_a, 2)
   *     joint_id (map):
   *         Key: joint name, Value: joint indicator
   *     floating_id (map):
   *         Key: floating joint name, Value: joint indicator
   *     link_id (map):
   *         Key: link name, Value: link indicator
   */
  virtual void _config_robot() = 0;
};
