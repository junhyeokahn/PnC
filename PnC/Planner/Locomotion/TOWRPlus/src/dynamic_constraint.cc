/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/******************************************************************************
Modified by Junhyeok Ahn (junhyeokahn91@gmail.com) for towr+
******************************************************************************/

#include <iostream>
#include <util/util.hpp>

#include <towr_plus/constraints/dynamic_constraint.h>
#include <towr_plus/models/composite_rigid_body_dynamics.h>
#include <towr_plus/variables/cartesian_dimensions.h>
#include <towr_plus/variables/variable_names.h>

namespace towr_plus {

DynamicConstraint::DynamicConstraint(const DynamicModel::Ptr &m, double T,
                                     double dt,
                                     const SplineHolder &spline_holder)
    : TimeDiscretizationConstraint(T, dt, "dynamic") {

  model_ = m;
  std::shared_ptr<CompositeRigidBodyInertia> crbi =
      std::static_pointer_cast<CompositeRigidBodyDynamics>(model_)->GetCRBI();
  crbi_helper_ = std::make_shared<CRBIHelper>(
      crbi, spline_holder.base_linear_, spline_holder.ee_motion_linear_[0],
      spline_holder.ee_motion_linear_[1]);

  // link with up-to-date spline variables
  base_linear_ = spline_holder.base_linear_;
  base_angular_ = EulerConverter(spline_holder.base_angular_);
  ee_wrench_linear_ = spline_holder.ee_wrench_linear_;
  ee_wrench_angular_ = spline_holder.ee_wrench_angular_;
  ee_motion_linear_ = spline_holder.ee_motion_linear_;
  ee_motion_angular_.resize(spline_holder.ee_motion_angular_.size());
  for (int i = 0; i < ee_motion_angular_.size(); ++i) {
    ee_motion_angular_[i] = EulerConverter(spline_holder.ee_motion_angular_[i]);
  }

  SetRows(GetNumberOfNodes() * k6D);
}

int DynamicConstraint::GetRow(int k, Dim6D dimension) const {
  return k6D * k + dimension;
}

void DynamicConstraint::UpdateConstraintAtInstance(double t, int k,
                                                   VectorXd &g) const {
  UpdateModel(t);
  g.segment(GetRow(k, AX), k6D) = model_->GetDynamicViolation();
}

void DynamicConstraint::UpdateBoundsAtInstance(double t, int k,
                                               VecBound &bounds) const {
  for (auto dim : AllDim6D)
    bounds.at(GetRow(k, dim)) = ifopt::BoundZero;
}

void DynamicConstraint::UpdateJacobianAtInstance(double t, int k,
                                                 std::string var_set,
                                                 Jacobian &jac) const {
  UpdateModel(t);

  int n = jac.cols();
  Jacobian jac_model(k6D, n);

  // sensitivity of dynamic constraint w.r.t base variables.
  if (var_set == id::base_lin_nodes) {
    Jacobian jac_base_lin_pos = base_linear_->GetJacobianWrtNodes(t, kPos);
    Jacobian jac_base_lin_acc = base_linear_->GetJacobianWrtNodes(t, kAcc);

    jac_model = model_->GetJacobianWrtBaseLin(
        jac_base_lin_pos, jac_base_lin_acc, t, crbi_helper_);
  }

  if (var_set == id::base_ang_nodes) {
    jac_model = model_->GetJacobianWrtBaseAng(base_angular_, t);
  }

  // sensitivity of dynamic constraint w.r.t. endeffector variables
  for (int ee = 0; ee < model_->GetEECount(); ++ee) {
    if (var_set == id::EEWrenchLinNodes(ee)) {
      Jacobian jac_ee_force =
          ee_wrench_linear_.at(ee)->GetJacobianWrtNodes(t, kPos);
      jac_model = model_->GetJacobianWrtForce(jac_ee_force, ee);
    }

    if (var_set == id::EEWrenchAngNodes(ee)) {
      Jacobian jac_ee_wrench =
          ee_wrench_angular_.at(ee)->GetJacobianWrtNodes(t, kPos);
      jac_model = model_->GetJacobianWrtTrq(jac_ee_wrench);
    }

    if (var_set == id::EEMotionLinNodes(ee)) {
      Jacobian jac_ee_pos =
          ee_motion_linear_.at(ee)->GetJacobianWrtNodes(t, kPos);
      jac_model = model_->GetJacobianWrtEEPos(jac_ee_pos, ee, t, crbi_helper_);
    }

    if (var_set == id::EESchedule(ee)) {
      Jacobian jac_f_dT =
          ee_wrench_linear_.at(ee)->GetJacobianOfPosWrtDurations(t);
      jac_model += model_->GetJacobianWrtForce(jac_f_dT, ee);

      Jacobian jac_x_dT =
          ee_motion_linear_.at(ee)->GetJacobianOfPosWrtDurations(t);
      jac_model +=
          model_->GetJacobianWrtEEPosSchedule(jac_x_dT, ee, t, crbi_helper_);

      Jacobian jac_trq_dT =
          ee_wrench_angular_.at(ee)->GetJacobianOfPosWrtDurations(t);
      jac_model += model_->GetJacobianWrtTrq(jac_trq_dT);
    }
  }

  // std::cout << "\n-------------------------------------\n" << std::endl;
  // std::cout << var_set << std::endl;
  // for (int i = 0; i < jac_model.outerSize(); ++i) {
  // for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
  // jac_model, i);
  // it; ++it) {
  // std::cout << "(" << it.row() << "," << it.col() << ") \t";
  //}
  //}
  // std::cout << "jac col : " << jac.cols() << std::endl;

  jac.middleRows(GetRow(k, AX), k6D) = jac_model;
}

void DynamicConstraint::UpdateModel(double t) const {
  auto com = base_linear_->GetPoint(t);

  Eigen::Matrix3d w_R_b = base_angular_.GetRotationMatrixBaseToWorld(t);
  Eigen::Vector3d omega = base_angular_.GetAngularVelocityInWorld(t);
  Eigen::Vector3d omega_dot = base_angular_.GetAngularAccelerationInWorld(t);

  int n_ee = model_->GetEECount();
  std::vector<Eigen::Vector3d> ee_pos;
  std::vector<Eigen::Matrix3d> ee_ori;
  std::vector<Eigen::Vector3d> ee_force;
  std::vector<Eigen::Vector3d> ee_trq;
  for (int ee = 0; ee < n_ee; ++ee) {
    ee_force.push_back(ee_wrench_linear_.at(ee)->GetPoint(t).p());
    ee_trq.push_back(ee_wrench_angular_.at(ee)->GetPoint(t).p());
    ee_pos.push_back(ee_motion_linear_.at(ee)->GetPoint(t).p());
    ee_ori.push_back(ee_motion_angular_.at(ee).GetRotationMatrixBaseToWorld(t));
  }

  model_->SetCurrent(com.p(), com.a(), w_R_b, omega, omega_dot, ee_force,
                     ee_trq, ee_pos, ee_ori);
}

} /* namespace towr_plus */
