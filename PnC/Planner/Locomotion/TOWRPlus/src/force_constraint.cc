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

#include <towr_plus/constraints/force_constraint.h>
#include <towr_plus/variables/variable_names.h>
#include <util/util.hpp>

#include <iostream>

namespace towr_plus {

ForceConstraint::ForceConstraint(const HeightMap::Ptr &terrain,
                                 double force_limit, double x, double y, EE ee,
                                 const SplineHolder &spline_holder)
    : ifopt::ConstraintSet(kSpecifyLater, "wrench-" + std::to_string(ee)) {
  terrain_ = terrain;
  fn_max_ = force_limit;
  mu_ = terrain->GetFrictionCoeff();
  ee_ = ee;

  x_ = x;
  y_ = y;
  ee_motion_angular_ = EulerConverter(spline_holder.ee_motion_angular_.at(ee));
  phase_durations_ = spline_holder.phase_durations_.at(ee);

  Uf_ = Eigen::MatrixXd::Zero(17, 3);
  Utau_ = Eigen::MatrixXd::Zero(17, 3);
  _set_U();

  n_constraints_per_node_ = 17;
}

void ForceConstraint::InitVariableDependedQuantities(const VariablesPtr &x) {
  ee_force_ =
      x->GetComponent<NodesVariablesPhaseBased>(id::EEWrenchLinNodes(ee_));
  ee_trq_ =
      x->GetComponent<NodesVariablesPhaseBased>(id::EEWrenchAngNodes(ee_));
  ee_motion_lin_ =
      x->GetComponent<NodesVariablesPhaseBased>(id::EEMotionLinNodes(ee_));
  ee_motion_ang_ =
      x->GetComponent<NodesVariablesPhaseBased>(id::EEMotionAngNodes(ee_));

  pure_stance_force_node_ids_ = ee_force_->GetIndicesOfNonConstantNodes();

  int constraint_count =
      pure_stance_force_node_ids_.size() * n_constraints_per_node_;
  SetRows(constraint_count);
}

Eigen::VectorXd ForceConstraint::GetValues() const {
  VectorXd g(GetRows());
  int idx(0);
  auto force_nodes = ee_force_->GetNodes();
  auto trq_nodes = ee_trq_->GetNodes();
  for (int f_node_id : pure_stance_force_node_ids_) {
    int phase = ee_force_->GetPhase(f_node_id);
    Eigen::Vector3d euler_xyz = ee_motion_ang_->GetValueAtStartOfPhase(phase);
    Eigen::Matrix3d w_R_ee = euler_xyz_to_rot(euler_xyz);

    Vector3d f = force_nodes.at(f_node_id).p();
    Vector3d tau = trq_nodes.at(f_node_id).p();

    Eigen::VectorXd val =
        Uf_ * w_R_ee.transpose() * f + Utau_ * w_R_ee.transpose() * tau;
    g.segment(idx * n_constraints_per_node_, n_constraints_per_node_) = val;
    idx += 1;
  }

  return g;
}

ForceConstraint::VecBound ForceConstraint::GetBounds() const {
  VecBound bounds;

  for (int i = 0;
       i < pure_stance_force_node_ids_.size() * n_constraints_per_node_; ++i) {
    bounds.push_back(ifopt::BoundGreaterZero);
  }

  return bounds;
}

void ForceConstraint::FillJacobianBlock(std::string var_set,
                                        Jacobian &jac) const {
  if (var_set == ee_force_->GetName()) {
    int idx = 0;
    for (int f_node_id : pure_stance_force_node_ids_) {
      int phase = ee_force_->GetPhase(f_node_id);
      Eigen::Vector3d euler_xyz = ee_motion_ang_->GetValueAtStartOfPhase(phase);
      Eigen::Matrix3d w_R_ee = euler_xyz_to_rot(euler_xyz);
      Eigen::MatrixXd Uf_times_ee_R_w = Uf_ * w_R_ee.transpose();

      int row = n_constraints_per_node_ * idx;
      for (int i = 0; i < n_constraints_per_node_; ++i) {
        for (auto dim : {X, Y, Z}) {
          int idx = ee_force_->GetOptIndex(
              NodesVariables::NodeValueInfo(f_node_id, kPos, dim));
          jac.coeffRef(row, idx) = Uf_times_ee_R_w(i, dim);
        }
        row += 1;
      }
      idx += 1;
    }
  }

  if (var_set == ee_trq_->GetName()) {
    int idx = 0;
    for (int f_node_id : pure_stance_force_node_ids_) {
      int phase = ee_trq_->GetPhase(f_node_id);
      Eigen::Vector3d euler_xyz = ee_motion_ang_->GetValueAtStartOfPhase(phase);
      Eigen::Matrix3d w_R_ee = euler_xyz_to_rot(euler_xyz);
      Eigen::MatrixXd Utau_times_ee_R_w = Utau_ * w_R_ee.transpose();

      int row = n_constraints_per_node_ * idx;
      for (int i = 0; i < n_constraints_per_node_; ++i) {
        for (auto dim : {X, Y, Z}) {
          int idx = ee_trq_->GetOptIndex(
              NodesVariables::NodeValueInfo(f_node_id, kPos, dim));
          jac.coeffRef(row, idx) = Utau_times_ee_R_w(i, dim);
        }
        row += 1;
      }
      idx += 1;
    }
  }

  if (var_set == ee_motion_ang_->GetName()) {
    int idx = 0;
    auto force_nodes = ee_force_->GetNodes();
    auto trq_nodes = ee_trq_->GetNodes();

    for (int f_node_id : pure_stance_force_node_ids_) {
      int phase = ee_force_->GetPhase(f_node_id);
      double t(0.);
      std::vector<double> pds = phase_durations_->GetPhaseDurations();
      for (int i = 0; i < pds.size(); ++i) {
        if (i < phase) {
          t += pds[i];
        }
      }

      Vector3d frc = force_nodes.at(f_node_id).p();
      Vector3d tau = trq_nodes.at(f_node_id).p();

      Jacobian jac1, jac2;
      jac1 =
          Uf_.sparseView() * ee_motion_angular_.DerivOfRotVecMult(t, frc, true);
      jac2 = Utau_.sparseView() *
             ee_motion_angular_.DerivOfRotVecMult(t, tau, true);

      jac.middleRows(n_constraints_per_node_ * idx, n_constraints_per_node_) =
          jac1 + jac2;
      idx += 1;
    }
  }
}

void ForceConstraint::_set_U() {
  Eigen::MatrixXd U = Eigen::MatrixXd::Zero(17, 6);
  U(0, 5) = 1.;

  U(1, 3) = 1.;
  U(1, 5) = mu_;
  U(2, 3) = -1.;
  U(2, 5) = mu_;

  U(3, 4) = 1.;
  U(3, 5) = mu_;
  U(4, 4) = -1.;
  U(4, 5) = mu_;

  U(5, 0) = 1.;
  U(5, 5) = y_;
  U(6, 0) = -1.;
  U(6, 5) = y_;

  U(7, 1) = 1.;
  U(7, 5) = x_;
  U(8, 1) = -1.;
  U(8, 5) = x_;

  // Tau
  U(9, 0) = -mu_;
  U(9, 1) = -mu_;
  U(9, 2) = 1;
  U(9, 3) = y_;
  U(9, 4) = x_;
  U(9, 5) = (x_ + y_) * mu_;

  U(10, 0) = -mu_;
  U(10, 1) = mu_;
  U(10, 2) = 1;
  U(10, 3) = y_;
  U(10, 4) = -x_;
  U(10, 5) = (x_ + y_) * mu_;

  U(11, 0) = mu_;
  U(11, 1) = -mu_;
  U(11, 2) = 1;
  U(11, 3) = -y_;
  U(11, 4) = x_;
  U(11, 5) = (x_ + y_) * mu_;

  U(12, 0) = mu_;
  U(12, 1) = mu_;
  U(12, 2) = 1;
  U(12, 3) = -y_;
  U(12, 4) = -x_;
  U(12, 5) = (x_ + y_) * mu_;
  /////////////////////////////////////////////////
  U(13, 0) = -mu_;
  U(13, 1) = -mu_;
  U(13, 2) = -1;
  U(13, 3) = -y_;
  U(13, 4) = -x_;
  U(13, 5) = (x_ + y_) * mu_;

  U(14, 0) = -mu_;
  U(14, 1) = mu_;
  U(14, 2) = -1;
  U(14, 3) = -y_;
  U(14, 4) = x_;
  U(14, 5) = (x_ + y_) * mu_;

  U(15, 0) = mu_;
  U(15, 1) = -mu_;
  U(15, 2) = -1;
  U(15, 3) = y_;
  U(15, 4) = -x_;
  U(15, 5) = (x_ + y_) * mu_;

  U(16, 0) = mu_;
  U(16, 1) = mu_;
  U(16, 2) = -1;
  U(16, 3) = y_;
  U(16, 4) = x_;
  U(16, 5) = (x_ + y_) * mu_;

  Uf_ = U.block(0, 3, 17, 3);
  Utau_ = U.block(0, 0, 17, 3);
}

} /* namespace towr_plus */
