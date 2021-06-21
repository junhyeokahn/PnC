/******************************************************************************
Written by Junhyeok Ahn (junhyeokahn91@gmail.com) for towr+
******************************************************************************/

#pragma once

#include "dynamic_model.h"

class CompositeRigidBodyInertia;

namespace towr_plus {

class CompositeRigidBodyDynamics : public DynamicModel {
public:
  CompositeRigidBodyDynamics(double mass, int ee_count,
                             std::shared_ptr<CompositeRigidBodyInertia> crbi);

  virtual ~CompositeRigidBodyDynamics();

  BaseAcc GetDynamicViolation() const override;

  Jac GetJacobianWrtBaseLin(
      const Jac &jac_base_lin_pos, const Jac &jac_acc_base_lin, double t,
      std::shared_ptr<CRBIHelper> crbi_helper) const override;
  Jac GetJacobianWrtBaseAng(const EulerConverter &base_angular,
                            double t) const override;
  Jac GetJacobianWrtForce(const Jac &jac_force, EE) const override;

  Jac GetJacobianWrtEEPos(
      const Jac &jac_ee_pos, EE, double t,
      std::shared_ptr<CRBIHelper> crbi_helper) const override;

  Jac GetJacobianWrtTrq(const Jac &jac_trq) const override;

  Jac GetJacobianWrtEEPosSchedule(
      const Jac &jac_ee_pos, EE, double t,
      std::shared_ptr<CRBIHelper> crbi_helper) const override;

  std::shared_ptr<CompositeRigidBodyInertia> GetCRBI() { return crbi_; }

private:
  std::shared_ptr<CompositeRigidBodyInertia> crbi_;
};

} /* namespace towr_plus */
