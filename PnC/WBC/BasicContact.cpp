#include <PnC/WBC/BasicContact.hpp>

PointContact::PointContact(RobotSystem *robot, const std::string _target_id,
                           const double _mu)
    : Contact(robot, 3, _target_id, _mu) {
  util::PrettyConstructor(3, "Point Contact");
}

PointContact::~PointContact() {}

void PointContact::_update_jacobian() {
  jacobian =
      robot_->get_link_jacobian(target_id).block(dim, 0, dim, robot_->n_q_dot);
  jacobian_dot_q_dot =
      robot_->get_link_jacobian_dot_times_qdot(target_id).tail(3);
}

void PointContact::_update_cone_constraint() {

  cone_constraint_mat = Eigen::MatrixXd::Zero(6, dim);
  Eigen::MatrixXd rot(robot_->get_link_iso(target_id).linear().transpose());

  cone_constraint_mat(0, 2) = 1.; // Fz >= 0
  cone_constraint_mat(1, 0) = 1.;
  cone_constraint_mat(1, 2) = mu_;
  cone_constraint_mat(2, 0) = -1.;
  cone_constraint_mat(2, 2) = mu_;
  cone_constraint_mat(3, 1) = 1.;
  cone_constraint_mat(3, 2) = mu_;
  cone_constraint_mat(4, 1) = -1.;
  cone_constraint_mat(4, 2) = mu_;

  // Upper bound of vertical directional reaction force
  cone_constraint_mat(5, 2) = -1.; // -Fz >= -rf_z_max

  cone_constraint_mat *= rot;

  cone_constraint_vec = Eigen::VectorXd::Zero(6);
  cone_constraint_vec[5] = -rf_z_max;
}

SurfaceContact::SurfaceContact(RobotSystem *robot, const std::string _target_id,
                               const double _x, const double _y,
                               const double _mu)
    : Contact(robot, 6, _target_id, _mu) {
  util::PrettyConstructor(3, "Surface Contact");

  x_ = _x;
  y_ = _y;
}

SurfaceContact::~SurfaceContact() {}

void SurfaceContact::_update_jacobian() {
  jacobian = robot_->get_link_jacobian(target_id);
  jacobian_dot_q_dot = robot_->get_link_jacobian_dot_times_qdot(target_id);
}

void SurfaceContact::_update_cone_constraint() {
  cone_constraint_mat = Eigen::MatrixXd::Zero(16 + 2, dim);

  Eigen::MatrixXd rot = Eigen::MatrixXd::Zero(6, 6);
  rot.block(0, 0, 3, 3) = robot_->get_link_iso(target_id).linear().transpose();
  rot.block(3, 3, 3, 3) = robot_->get_link_iso(target_id).linear().transpose();

  cone_constraint_mat = this->_get_u(x_, y_, mu_);
  cone_constraint_mat *= rot;

  cone_constraint_vec = Eigen::VectorXd::Zero(16 + 2);
  cone_constraint_vec[17] = -rf_z_max;
}

Eigen::MatrixXd SurfaceContact::_get_u(double x, double y, double mu) {
  Eigen::MatrixXd U = Eigen::MatrixXd::Zero(16 + 2, 6);

  U(0, 5) = 1.;

  U(1, 3) = 1.;
  U(1, 5) = mu;
  U(2, 3) = -1.;
  U(2, 5) = mu;

  U(3, 4) = 1.;
  U(3, 5) = mu;
  U(4, 4) = -1.;
  U(4, 5) = mu;

  U(5, 0) = 1.;
  U(5, 5) = y;
  U(6, 0) = -1.;
  U(6, 5) = y;

  U(7, 1) = 1.;
  U(7, 5) = x;
  U(8, 1) = -1.;
  U(8, 5) = x;

  // Tau
  U(9, 0) = -mu;
  U(9, 1) = -mu;
  U(9, 2) = 1;
  U(9, 3) = y;
  U(9, 4) = x;
  U(9, 5) = (x + y) * mu;

  U(10, 0) = -mu;
  U(10, 1) = mu;
  U(10, 2) = 1;
  U(10, 3) = y;
  U(10, 4) = -x;
  U(10, 5) = (x + y) * mu;

  U(11, 0) = mu;
  U(11, 1) = -mu;
  U(11, 2) = 1;
  U(11, 3) = -y;
  U(11, 4) = x;
  U(11, 5) = (x + y) * mu;

  U(12, 0) = mu;
  U(12, 1) = mu;
  U(12, 2) = 1;
  U(12, 3) = -y;
  U(12, 4) = -x;
  U(12, 5) = (x + y) * mu;
  /////////////////////////////////////////////////
  U(13, 0) = -mu;
  U(13, 1) = -mu;
  U(13, 2) = -1;
  U(13, 3) = -y;
  U(13, 4) = -x;
  U(13, 5) = (x + y) * mu;

  U(14, 0) = -mu;
  U(14, 1) = mu;
  U(14, 2) = -1;
  U(14, 3) = -y;
  U(14, 4) = x;
  U(14, 5) = (x + y) * mu;

  U(15, 0) = mu;
  U(15, 1) = -mu;
  U(15, 2) = -1;
  U(15, 3) = y;
  U(15, 4) = -x;
  U(15, 5) = (x + y) * mu;

  U(16, 0) = mu;
  U(16, 1) = mu;
  U(16, 2) = -1;
  U(16, 3) = y;
  U(16, 4) = x;
  U(16, 5) = (x + y) * mu;
  // ////////////////////////////////////////////////////
  U(17, 5) = -1.;

  return U;
}
