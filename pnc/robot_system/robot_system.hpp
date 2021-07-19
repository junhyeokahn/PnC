#pragma once

#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>

/// class RobotSystem
class RobotSystem {
public:
  /// \{ \name Constructor and Destructor
  RobotSystem(const bool _b_fixed_base, const bool _b_print_info)
      : b_fixed_base(_b_fixed_base), b_print_info(_b_print_info){};

  virtual ~RobotSystem(){};
  /// \}

  bool b_fixed_base;
  bool b_print_info;

  /// Number of degree of freedom that represents floating base of the robot
  int n_floating;

  /// Number of positions in generalized coordinates.
  int n_q;

  /// Number of velocities in generalized coordinates.
  int n_q_dot;

  /// Number of actuation in generalized coorinates.
  int n_a;

  /// Total mass of the robot
  double total_mass;

  /// Position limits of the actuating joints (size of n_a). The first and the
  /// second row represent min and max values, respectively.
  Eigen::Matrix<double, Eigen::Dynamic, 2> joint_pos_limit;

  /// Velocity limits of the actuating joints (size of n_a). The first and the
  /// second row represent min and max values, respectively.
  Eigen::Matrix<double, Eigen::Dynamic, 2> joint_vel_limit;

  /// Torque limits of the actuating joints (size of n_a). The first and the
  /// second row represent min and max values, respectively.
  Eigen::Matrix<double, Eigen::Dynamic, 2> joint_trq_limit;

  /// Positions of the joints (size of n_a).
  Eigen::VectorXd joint_positions;

  /// Velocities of the joints (size of n_a).
  Eigen::VectorXd joint_velocities;

  /// Centroidal inertia tensor
  Eigen::Matrix<double, 6, 6> Ig;

  /// Centroidal momentum matrix
  Eigen::Matrix<double, 6, Eigen::Dynamic> Ag;

  /// Centroidal momentum
  Eigen::Matrix<double, 6, 1> hg;

  /// Get position index in generalized coordinate
  virtual int get_q_idx(const std::string joint_name) = 0;

  /// Get velocity index in generalized coordinate
  virtual int get_q_dot_idx(const std::string joint_name) = 0;

  /// Get joint index in generalized coordinate
  virtual int get_joint_idx(const std::string joint_name) = 0;

  /// Create map from eigen vector
  virtual std::map<std::string, double>
  vector_to_map(const Eigen::VectorXd &cmd_vec) = 0;

  /// Create eigen vector from map
  virtual Eigen::VectorXd map_to_vector(std::map<std::string, double>) = 0;

  /// Get relative pos of base com frame w.r.t. base joint frame
  virtual Eigen::Vector3d get_base_local_com_pos() = 0;

  /// Update robot with the sensor data
  //
  /// \param[in] base_com_pos Root link com frame's position with respect to the
  /// world frame.
  //
  /// \param[in] base_com_quat Root link com frame's quaternion with respect to
  /// the world frame.
  //
  /// \param[in] base_com_lin_vel Root link com frame's linear velocity with
  /// respect to the world frame.
  //
  /// \param[in] base_com_ang_vel Root link com frame's angular velocity with
  /// respect to the world frame.
  //
  /// \param[in] base_joint_pos Root link joint frame's position with respect to
  /// the world frame.
  //
  /// \param[in] base_joint_quat Root link joint frame's quaternion with respect
  /// to the world frame.
  //
  /// \param[in] base_joint_lin_vel Root link joint frame's linear velocity with
  /// respect to the world frame.
  //
  /// \param[in] base_joint_ang_vel Root link joint frame's angular velocity
  /// with respect to the world frame.
  //
  /// \param[in] joint_pos Map of joint name and position measurement.
  //
  /// \param[in] joint_vel Map of joint name and velocity measurement.
  //
  /// \param[in] b_cent Whether to update centroid quantities or not.
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

  /// Get positions in generalized coordinate (size of n_q).
  virtual Eigen::VectorXd get_q() = 0;

  /// Get velocities in generalized coordinate (size of n_q_dot).
  virtual Eigen::VectorXd get_q_dot() = 0;

  /// Get mass matrix in generalized coordinate
  virtual Eigen::MatrixXd get_mass_matrix() = 0;

  /// Get gravity forces in generalized coordinate
  virtual Eigen::VectorXd get_gravity() = 0;

  /// Get coriolis forces in generalized coordinate
  virtual Eigen::VectorXd get_coriolis() = 0;

  /// Get the robot's center of mass position with respect to the world frame.
  virtual Eigen::Vector3d get_com_pos() = 0;

  /// Get the robot's center of mass linear velocity with respect to the world
  /// frame.
  virtual Eigen::Vector3d get_com_lin_vel() = 0;

  /// Get the robot's center of mass linear jacobian with respect to the world
  /// frame.
  virtual Eigen::Matrix<double, 3, Eigen::Dynamic> get_com_lin_jacobian() = 0;

  /// Get the robot's center of mass linear jacobian dot with respect to the
  /// world frame.
  virtual Eigen::Matrix<double, 3, Eigen::Dynamic>
  get_com_lin_jacobian_dot() = 0;

  /// Get link's center of mass frame SE(3).
  virtual Eigen::Isometry3d get_link_iso(const std::string link_id) = 0;

  /// Get link's center of mass frame classic velocity.
  virtual Eigen::Matrix<double, 6, 1>
  get_link_vel(const std::string link_id) = 0;

  /// Get link's center of mass frame classic jacobian.
  virtual Eigen::Matrix<double, 6, Eigen::Dynamic>
  get_link_jacobian(const std::string link_id) = 0;

  /// Get link's center of mass frame classic jacobian dot.
  virtual Eigen::Matrix<double, 6, 1>
  get_link_jacobian_dot_times_qdot(const std::string link_id) = 0;

private:
  /// Update centroid quantities (Ig, Ag, hg).
  virtual void _update_centroidal_quantities() = 0;

  /// Configure the following properties: n_floating, n_q, n_q_dot, n_a,
  /// total_mass, joint_pos_limit, joint_vel_limit, joint_trq_limit.
  virtual void _config_robot() = 0;
};
