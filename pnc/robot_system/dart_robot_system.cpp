#include "pnc/robot_system/dart_robot_system.hpp"

#include <utils/util.hpp>

DartRobotSystem::DartRobotSystem(const std::string _urdf_file,
                                 const bool _b_fixed_base,
                                 const bool _b_print_info)
    : RobotSystem(_b_fixed_base, _b_print_info), urdf_file_(_urdf_file) {
  util::PrettyConstructor(1, "DartRobotSystem");
  this->_config_robot();

  joint_positions.resize(n_a);
  joint_velocities.resize(n_a);

  Ag.resize(6, n_q_dot);
}

DartRobotSystem::~DartRobotSystem() {}

void DartRobotSystem::_config_robot() {
  dart::utils::DartLoader urdfLoader;
  skel_ = urdfLoader.parseSkeleton(urdf_file_);

  if (b_fixed_base) {
    skel_->getRootBodyNode()
        ->changeParentJointType<dart::dynamics::WeldJoint>();
    n_floating = 0;
  }

  for (int i = 0; i < skel_->getNumJoints(); ++i) {
    dart::dynamics::JointPtr joint = skel_->getJoint(i);
    if (joint->getName() == "rootJoint") {
      n_floating = joint->getNumDofs();
    } else if (joint->getType() != "WeldJoint") {
      joint_id_[joint->getName()] = joint;
    } else {
    }
  }

  for (int i = 0; i < skel_->getNumBodyNodes(); ++i) {
    dart::dynamics::BodyNodePtr bn = skel_->getBodyNode(i);
    link_id_[bn->getName()] = bn;
  }

  n_q = skel_->getNumDofs();
  n_q_dot = skel_->getNumDofs();
  n_a = n_q - n_floating;
  total_mass = skel_->getMass();

  joint_pos_limit.resize(n_a, 2);
  joint_vel_limit.resize(n_a, 2);
  joint_trq_limit.resize(n_a, 2);

  joint_pos_limit.block(0, 0, n_a, 1) =
      skel_->getPositionLowerLimits().segment(n_floating, n_a);
  joint_pos_limit.block(0, 1, n_a, 1) =
      skel_->getPositionUpperLimits().segment(n_floating, n_a);
  joint_vel_limit.block(0, 0, n_a, 1) =
      skel_->getVelocityLowerLimits().segment(n_floating, n_a);
  joint_vel_limit.block(0, 1, n_a, 1) =
      skel_->getVelocityUpperLimits().segment(n_floating, n_a);
  joint_trq_limit.block(0, 0, n_a, 1) =
      skel_->getForceLowerLimits().segment(n_floating, n_a);
  joint_trq_limit.block(0, 1, n_a, 1) =
      skel_->getForceUpperLimits().segment(n_floating, n_a);

  if (b_print_info) {
    std::cout << "========================" << std::endl;
    std::cout << "PnC Robot" << std::endl;
    std::cout << "nq: " << n_q << std::endl;
    std::cout << "nv: " << n_q_dot << std::endl;
    std::cout << "na: " << n_a << std::endl;
    std::cout << "Joint Info: " << std::endl;
    int id(0);
    for (std::map<std::string, dart::dynamics::JointPtr>::iterator it =
             joint_id_.begin();
         it != joint_id_.end(); it++) {
      std::cout << id << ": " << it->first << std::endl;
      id += 1;
    }

    std::cout << "Link Info: " << std::endl;
    id = 0;
    for (std::map<std::string, dart::dynamics::BodyNodePtr>::iterator it =
             link_id_.begin();
         it != link_id_.end(); it++) {
      std::cout << id << ": " << it->first << std::endl;
      id += 1;
    }

    std::cout << "Dof Info: " << std::endl;
    for (int i = 0; i < skel_->getNumDofs(); ++i) {
      dart::dynamics::DegreeOfFreedom *dof = skel_->getDof(i);
      std::cout << i << " : " << dof->getName() << std::endl;
    }
  }
}

Eigen::Vector3d DartRobotSystem::get_base_local_com_pos() {
  return skel_->getRootBodyNode()->getLocalCOM();
}

std::string DartRobotSystem::get_base_link_name() {
  return skel_->getRootBodyNode()->getName();
}

int DartRobotSystem::get_q_idx(const std::string joint_name) {
  return joint_id_[joint_name]->getIndexInSkeleton(0);
}

int DartRobotSystem::get_q_dot_idx(const std::string joint_name) {
  return joint_id_[joint_name]->getIndexInSkeleton(0);
}

int DartRobotSystem::get_joint_idx(const std::string joint_name) {
  return joint_id_[joint_name]->getIndexInSkeleton(0) - n_floating;
}

std::map<std::string, double>
DartRobotSystem::vector_to_map(const Eigen::VectorXd &cmd_vec) {
  std::map<std::string, double> ret;

  for (std::map<std::string, dart::dynamics::JointPtr>::iterator it =
           joint_id_.begin();
       it != joint_id_.end(); it++) {
    int joint_id = get_joint_idx(it->first);
    ret[it->first] = cmd_vec[joint_id];
  }

  return ret;
}

Eigen::VectorXd
DartRobotSystem::map_to_vector(std::map<std::string, double> _map) {
  Eigen::VectorXd vec = Eigen::VectorXd::Zero(joint_positions.size());

  for (std::map<std::string, double>::iterator it = _map.begin();
       it != _map.end(); it++) {
    int joint_id = get_joint_idx(it->first);
    vec[joint_id] = it->second;
  }

  return vec;
}

void DartRobotSystem::update_system(
    const Eigen::Vector3d base_com_pos,
    const Eigen::Quaternion<double> base_com_quat,
    const Eigen::Vector3d base_com_lin_vel,
    const Eigen::Vector3d base_com_ang_vel,
    const Eigen::Vector3d base_joint_pos,
    const Eigen::Quaternion<double> base_joint_quat,
    const Eigen::Vector3d base_joint_lin_vel,
    const Eigen::Vector3d base_joint_ang_vel,
    const std::map<std::string, double> joint_pos,
    const std::map<std::string, double> joint_vel, const bool b_cent) {
  if (!b_fixed_base) {
    // Floating Base Robot
    Eigen::Isometry3d joint_iso;
    joint_iso.linear() = base_joint_quat.normalized().toRotationMatrix();
    joint_iso.translation() = base_joint_pos;
    Eigen::Matrix<double, 6, 1> joint_vel_in_world;
    joint_vel_in_world.segment(0, 3) = base_joint_ang_vel;
    joint_vel_in_world.segment(3, 3) = base_joint_lin_vel;
    Eigen::Matrix<double, 6, 1> zero_acc = Eigen::Matrix<double, 6, 1>::Zero();
    dynamic_cast<dart::dynamics::FreeJoint *>(skel_->getRootJoint())
        ->setSpatialMotion(&joint_iso, dart::dynamics::Frame::World(),
                           &joint_vel_in_world, dart::dynamics::Frame::World(),
                           dart::dynamics::Frame::World(), &zero_acc,
                           dart::dynamics::Frame::World(),
                           dart::dynamics::Frame::World());

  } else {
    // Fixed Base Robot
    Eigen::Isometry3d base_joint_iso;
    base_joint_iso.linear() = base_com_quat.normalized().toRotationMatrix();
    base_joint_iso.translation() = base_joint_pos;

    skel_->getRootJoint()->setTransformFromParentBodyNode(base_joint_iso);
  }

  for (std::map<std::string, double>::const_iterator it = joint_pos.begin();
       it != joint_pos.end(); it++) {
    joint_id_.find(it->first)->second->setPosition(0, it->second);
    joint_positions[this->get_joint_idx(it->first)] = it->second;
  }

  for (std::map<std::string, double>::const_iterator it = joint_vel.begin();
       it != joint_vel.end(); it++) {
    joint_id_.find(it->first)->second->setVelocity(0, it->second);
    joint_velocities[this->get_joint_idx(it->first)] = it->second;
  }

  skel_->computeForwardKinematics();
  skel_->computeForwardDynamics();

  if (b_cent) {
    this->_update_centroidal_quantities();
  }
}

void DartRobotSystem::_update_centroidal_quantities() {
  Eigen::Matrix<double, 6, Eigen::Dynamic> Jsp;
  Eigen::Vector3d p_gl;
  Eigen::Matrix3d R_gl;
  Eigen::Vector3d pCoM_g;
  Eigen::Matrix<double, 6, 6> I;
  Eigen::Isometry3d T_lc = Eigen::Isometry3d::Identity();
  Eigen::Matrix<double, 6, 6> AdT_lc;
  Ig.setZero();
  Ag.setZero();
  pCoM_g = skel_->getCOM();
  for (int i = 0; i < skel_->getNumBodyNodes(); ++i) {
    dart::dynamics::BodyNodePtr bn = skel_->getBodyNode(i);
    Jsp = skel_->getJacobian(bn);
    p_gl = bn->getWorldTransform().translation();
    R_gl = bn->getWorldTransform().linear();
    I = bn->getSpatialInertia();
    T_lc.linear() = R_gl.transpose();
    T_lc.translation() = R_gl.transpose() * (pCoM_g - p_gl);
    AdT_lc = dart::math::getAdTMatrix(T_lc);
    Ig += AdT_lc.transpose() * I * AdT_lc;
    Ag += AdT_lc.transpose() * I * Jsp;
  }
  hg = Ag * get_q_dot();
}

Eigen::VectorXd DartRobotSystem::get_q() { return skel_->getPositions(); }

Eigen::VectorXd DartRobotSystem::get_q_dot() { return skel_->getVelocities(); }

Eigen::MatrixXd DartRobotSystem::get_mass_matrix() {
  return skel_->getMassMatrix();
}

Eigen::VectorXd DartRobotSystem::get_gravity() {
  return skel_->getGravityForces();
}

Eigen::VectorXd DartRobotSystem::get_coriolis() {
  return skel_->getCoriolisForces();
}

Eigen::Vector3d DartRobotSystem::get_com_pos() {
  return skel_->getCOM(dart::dynamics::Frame::World());
}

Eigen::Vector3d DartRobotSystem::get_com_lin_vel() {
  return skel_->getCOMLinearVelocity(dart::dynamics::Frame::World(),
                                     dart::dynamics::Frame::World());
}

Eigen::Matrix<double, 3, Eigen::Dynamic>
DartRobotSystem::get_com_lin_jacobian() {
  return skel_->getCOMLinearJacobian(dart::dynamics::Frame::World());
}

Eigen::Matrix<double, 3, Eigen::Dynamic>
DartRobotSystem::get_com_lin_jacobian_dot() {
  return skel_->getCOMLinearJacobianDeriv(dart::dynamics::Frame::World());
}

Eigen::Isometry3d DartRobotSystem::get_link_iso(const std::string link_id) {
  Eigen::Isometry3d ret = link_id_[link_id]->getTransform(
      dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
  ret.translation() = link_id_[link_id]->getCOM(dart::dynamics::Frame::World());
  return ret;
}

Eigen::Matrix<double, 6, 1>
DartRobotSystem::get_link_vel(const std::string link_id) {
  return link_id_[link_id]->getCOMSpatialVelocity(
      dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
}

Eigen::Matrix<double, 6, Eigen::Dynamic>
DartRobotSystem::get_link_jacobian(const std::string link_id) {
  return skel_->getJacobian(link_id_[link_id], link_id_[link_id]->getLocalCOM(),
                            dart::dynamics::Frame::World());
}

Eigen::Matrix<double, 6, 1>
DartRobotSystem::get_link_jacobian_dot_times_qdot(const std::string link_id) {
  return skel_->getJacobianClassicDeriv(link_id_[link_id],
                                        link_id_[link_id]->getLocalCOM(),
                                        dart::dynamics::Frame::World()) *
         get_q_dot();
}
