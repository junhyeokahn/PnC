#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

#include "pnc/robot_system/robot_system.hpp"

/*
 *  Dart considers floating base with 6 positions and 6 velocities with the
 *  order of rotx, roty, rotz, x, y, z. Therefore, n_q = n_v
 *  Note that the joint named with 'rootJoint' has 6 dof to represent the
 *  floating base.
 */

/// class DartRobotSystem
class DartRobotSystem : public RobotSystem {
public:
  /// \{ \name Constructor and Destructor
  DartRobotSystem(const std::string _urdf_file, const bool _b_fixed_base,
                  const bool _b_print_info = false);
  virtual ~DartRobotSystem();
  /// \}

  virtual int get_q_idx(const std::string joint_name);
  virtual int get_q_dot_idx(const std::string joint_name);
  virtual int get_joint_idx(const std::string joint_name);
  virtual std::map<std::string, double>
  vector_to_map(const Eigen::VectorXd &cmd_vec);
  virtual Eigen::VectorXd map_to_vector(std::map<std::string, double>);
  virtual Eigen::Vector3d get_base_local_com_pos();
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
                             const bool b_cent = false);
  virtual Eigen::VectorXd get_q();
  virtual Eigen::VectorXd get_q_dot();
  virtual Eigen::MatrixXd get_mass_matrix();
  virtual Eigen::VectorXd get_gravity();
  virtual Eigen::VectorXd get_coriolis();
  virtual Eigen::Vector3d get_com_pos();
  virtual Eigen::Vector3d get_com_lin_vel();
  virtual Eigen::Matrix<double, 3, Eigen::Dynamic> get_com_lin_jacobian();
  virtual Eigen::Matrix<double, 3, Eigen::Dynamic> get_com_lin_jacobian_dot();
  virtual Eigen::Isometry3d get_link_iso(const std::string link_id);
  virtual Eigen::Vector6d get_link_vel(const std::string link_id);
  virtual Eigen::Matrix<double, 6, Eigen::Dynamic>
  get_link_jacobian(const std::string link_id);
  virtual Eigen::Matrix<double, 6, 1>
  get_link_jacobian_dot_times_qdot(const std::string link_id);

private:
  virtual void _update_centroidal_quantities();
  virtual void _config_robot();

  dart::dynamics::SkeletonPtr skel_;
  std::string urdf_file_;

  /// Map of joint name and dart's JointPtr
  std::map<std::string, dart::dynamics::JointPtr> joint_id_;

  /// Map of link name and dart's BodyNodePtr
  std::map<std::string, dart::dynamics::BodyNodePtr> link_id_;
};
