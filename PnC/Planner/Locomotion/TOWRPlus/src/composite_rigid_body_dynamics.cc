/******************************************************************************
Written by Junhyeok Ahn (junhyeokahn91@gmail.com) for towr+
******************************************************************************/

#include <util/util.hpp>

#include <towr_plus/models/composite_rigid_body_dynamics.h>
#include <towr_plus/models/composite_rigid_body_inertia.h>
#include <towr_plus/variables/cartesian_dimensions.h>

namespace towr_plus {

// builds a cross product matrix out of "in", so in x v = X(in)*v
CompositeRigidBodyDynamics::Jac Cross(const Eigen::Vector3d &in) {
  CompositeRigidBodyDynamics::Jac out(3, 3);

  out.coeffRef(0, 1) = -in(2);
  out.coeffRef(0, 2) = in(1);
  out.coeffRef(1, 0) = in(2);
  out.coeffRef(1, 2) = -in(0);
  out.coeffRef(2, 0) = -in(1);
  out.coeffRef(2, 1) = in(0);

  return out;
}

CompositeRigidBodyDynamics::CompositeRigidBodyDynamics(
    double mass, int ee_count, std::shared_ptr<CompositeRigidBodyInertia> crbi)
    : DynamicModel(mass, ee_count) {

  crbi_ = crbi;
}

CompositeRigidBodyDynamics::~CompositeRigidBodyDynamics() {}

CompositeRigidBodyDynamics::BaseAcc
CompositeRigidBodyDynamics::GetDynamicViolation() const {
  // https://en.wikipedia.org/wiki/Newton%E2%80%93Euler_equations

  Vector3d f_sum, tau_sum;
  f_sum.setZero();
  tau_sum.setZero();

  for (int ee = 0; ee < ee_pos_.size(); ++ee) {
    Vector3d f = ee_force_.at(ee);
    tau_sum += f.cross(com_pos_ - ee_pos_.at(ee)) + ee_trq_.at(ee);
    f_sum += f;
  }

  // express inertia matrix in world frame based on current body orientation
  Jac I_b =
      (crbi_->ComputeInertia(com_pos_, ee_pos_[0], ee_pos_[1])).sparseView();
  Jac I_w = w_R_b_.sparseView() * I_b * w_R_b_.transpose().sparseView();

  BaseAcc acc;
  acc.segment(AX, k3D) =
      I_w * omega_dot_ + Cross(omega_) * (I_w * omega_) - tau_sum;
  acc.segment(LX, k3D) =
      m() * com_acc_ - f_sum - Vector3d(0.0, 0.0, -m() * g()); // gravity force
  return acc;
}

CompositeRigidBodyDynamics::Jac
CompositeRigidBodyDynamics::GetJacobianWrtBaseLin(
    const Jac &jac_pos_base_lin, const Jac &jac_acc_base_lin, double t,
    std::shared_ptr<CRBIHelper> crbi_helper) const {
  // build the com jacobian
  int n = jac_pos_base_lin.cols();

  Eigen::Vector3d v11 = w_R_b_.transpose() * omega_dot_;
  Jac jac11 =
      crbi_helper->GetDerivativeOfInertiaMatrixWrtBaseLinNodesMult(t, v11);
  Jac jac1 = w_R_b_.sparseView() * jac11;

  Eigen::Vector3d v21 = w_R_b_.transpose() * omega_;
  Jac jac21 =
      crbi_helper->GetDerivativeOfInertiaMatrixWrtBaseLinNodesMult(t, v21);
  Jac jac2 = Cross(omega_) * w_R_b_.sparseView() * jac21;

  Jac jac_tau_sum(k3D, n);
  for (const Vector3d &f : ee_force_) {
    Jac jac_tau = Cross(f) * jac_pos_base_lin;
    jac_tau_sum += jac_tau;
  }

  Jac jac(k6D, n);
  jac.middleRows(AX, k3D) = jac1 + jac2 - jac_tau_sum;
  jac.middleRows(LX, k3D) = m() * jac_acc_base_lin;

  return jac;
}

CompositeRigidBodyDynamics::Jac
CompositeRigidBodyDynamics::GetJacobianWrtBaseAng(
    const EulerConverter &base_euler, double t) const {
  Jac I_b =
      (crbi_->ComputeInertia(com_pos_, ee_pos_[0], ee_pos_[1])).sparseView();
  Jac I_w = w_R_b_.sparseView() * I_b * w_R_b_.transpose().sparseView();

  // Derivative of R*I_b*R^T * wd
  // 1st term of product rule (derivative of R)
  Vector3d v11 = I_b * w_R_b_.transpose() * omega_dot_;
  Jac jac11 = base_euler.DerivOfRotVecMult(t, v11, false);

  // 2nd term of product rule (derivative of R^T)
  Jac jac12 = w_R_b_.sparseView() * I_b *
              base_euler.DerivOfRotVecMult(t, omega_dot_, true);

  // 3rd term of product rule (derivative of wd)
  Jac jac_ang_acc = base_euler.GetDerivOfAngAccWrtEulerNodes(t);
  Jac jac13 = I_w * jac_ang_acc;
  Jac jac1 = jac11 + jac12 + jac13;

  // Derivative of w x Iw
  // w x d_dn(R*I_b*R^T*w) -(I*w x d_dnw)
  // right derivative same as above, just with velocity instead acceleration
  Vector3d v21 = I_b * w_R_b_.transpose() * omega_;
  Jac jac21 = base_euler.DerivOfRotVecMult(t, v21, false);

  // 2nd term of product rule (derivative of R^T)
  Jac jac22 =
      w_R_b_.sparseView() * I_b * base_euler.DerivOfRotVecMult(t, omega_, true);

  // 3rd term of product rule (derivative of omega)
  Jac jac_ang_vel = base_euler.GetDerivOfAngVelWrtEulerNodes(t);
  Jac jac23 = I_w * jac_ang_vel;

  Jac jac2 = Cross(omega_) * (jac21 + jac22 + jac23) -
             Cross(I_w * omega_) * jac_ang_vel;

  // Combine the two to get sensitivity to I_w*w + w x (I_w*w)
  int n = jac_ang_vel.cols();
  Jac jac(k6D, n);
  jac.middleRows(AX, k3D) = jac1 + jac2;

  return jac;
}

CompositeRigidBodyDynamics::Jac
CompositeRigidBodyDynamics::GetJacobianWrtForce(const Jac &jac_force,
                                                EE ee) const {
  Vector3d r = com_pos_ - ee_pos_.at(ee);
  Jac jac_tau = -Cross(r) * jac_force;

  int n = jac_force.cols();
  Jac jac(k6D, n);
  jac.middleRows(AX, k3D) = -jac_tau;
  jac.middleRows(LX, k3D) = -jac_force;

  return jac;
}

CompositeRigidBodyDynamics::Jac
CompositeRigidBodyDynamics::GetJacobianWrtTrq(const Jac &jac_trq) const {
  int n = jac_trq.cols();
  Jac jac(k6D, n);
  jac.middleRows(AX, k3D) = -jac_trq;

  return jac;
}

CompositeRigidBodyDynamics::Jac CompositeRigidBodyDynamics::GetJacobianWrtEEPos(
    const Jac &jac_ee_pos, EE ee, double t,
    std::shared_ptr<CRBIHelper> crbi_helper) const {

  Eigen::Vector3d v11 = w_R_b_.transpose() * omega_dot_;
  Jac jac11 =
      crbi_helper->GetDerivativeOfInertiaMatrixWrtEELinNodesMult(t, v11, ee);
  Jac jac1 = w_R_b_.sparseView() * jac11;

  Eigen::Vector3d v21 = w_R_b_.transpose() * omega_;
  Jac jac21 =
      crbi_helper->GetDerivativeOfInertiaMatrixWrtEELinNodesMult(t, v21, ee);
  Jac jac2 = Cross(omega_) * w_R_b_.sparseView() * jac21;

  Vector3d f = ee_force_.at(ee);
  Jac jac_tau = Cross(f) * (-jac_ee_pos);

  Jac jac(k6D, jac_tau.cols());
  jac.middleRows(AX, k3D) = jac1 + jac2 - jac_tau;

  // linear dynamics don't depend on endeffector position.
  return jac;
}

CompositeRigidBodyDynamics::Jac
CompositeRigidBodyDynamics::GetJacobianWrtEEPosSchedule(
    const Jac &jac_ee_pos, EE ee, double t,
    std::shared_ptr<CRBIHelper> crbi_helper) const {
  Eigen::Vector3d v11 = w_R_b_.transpose() * omega_dot_;
  Jac jac11 = crbi_helper->GetDerivativeOfInertiaMatrixWrtEEScheduleNodesMult(
      t, v11, ee);
  Jac jac1 = w_R_b_.sparseView() * jac11;

  Eigen::Vector3d v21 = w_R_b_.transpose() * omega_;
  Jac jac21 = crbi_helper->GetDerivativeOfInertiaMatrixWrtEEScheduleNodesMult(
      t, v21, ee);
  Jac jac2 = Cross(omega_) * w_R_b_.sparseView() * jac21;

  Vector3d f = ee_force_.at(ee);
  Jac jac_tau = Cross(f) * (-jac_ee_pos);

  Jac jac(k6D, jac_tau.cols());
  jac.middleRows(AX, k3D) = jac1 + jac2 - jac_tau;

  // linear dynamics don't depend on endeffector position.
  return jac;
}
} /* namespace towr_plus */
