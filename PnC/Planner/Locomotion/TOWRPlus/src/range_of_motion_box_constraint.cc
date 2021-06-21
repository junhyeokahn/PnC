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

#include <towr_plus/constraints/range_of_motion_box_constraint.h>
#include <towr_plus/variables/variable_names.h>

namespace towr_plus {

RangeOfMotionBoxConstraint::RangeOfMotionBoxConstraint(
    const KinematicModel::Ptr &model, double T, double dt, const EE &ee,
    const SplineHolder &spline_holder)
    : TimeDiscretizationConstraint(T, dt, "box_rom-" + std::to_string(ee)) {
  base_linear_ = spline_holder.base_linear_;
  base_angular_ = EulerConverter(spline_holder.base_angular_);
  ee_motion_linear_ = spline_holder.ee_motion_linear_.at(ee);
  ee_motion_angular_ = EulerConverter(spline_holder.ee_motion_angular_.at(ee));

  max_deviation_from_nominal_ = model->GetMaximumDeviationFromNominal();
  min_deviation_from_nominal_ = model->GetMinimumDeviationFromNominal();
  max_cos_ = cos(0.);
  min_cos_ = cos(0.175); // << 10 deg
  // min_cos_ = cos(0.261); // << 15 deg
  // min_cos_ = cos(0.785); // << 45 deg
  nominal_ee_pos_B_ = model->GetNominalStanceInBase().at(ee);
  ee_ = ee;

  S1_ = Eigen::MatrixXd::Zero(2, 3);
  S1_(0, 0) = 1.0;
  S1_(1, 1) = 1.0;
  S1__ = S1_.sparseView();
  S2_ = Eigen::MatrixXd::Zero(3, 1);
  S2_(0, 0) = 1.0;
  S2__ = S2_.sparseView();
  local_x_ = Eigen::MatrixXd::Zero(2, 1);
  local_x_(0) = 1.0;
  local_x__ = local_x_.sparseView();

  SetRows(GetNumberOfNodes() * 4); // X, Y, Z, Rot
}

int RangeOfMotionBoxConstraint::GetRow(int node, int dim) const {
  // dim should be in [0, 1, 2, 3]
  if (3 < dim || dim < 0) {
    assert(false);
  }
  return node * 4 + dim;
}

void RangeOfMotionBoxConstraint::UpdateConstraintAtInstance(double t, int k,
                                                            VectorXd &g) const {

  Vector3d base_W = base_linear_->GetPoint(t).p();
  Vector3d pos_ee_W = ee_motion_linear_->GetPoint(t).p();

  EulerConverter::MatrixSXd w_R_b =
      base_angular_.GetRotationMatrixBaseToWorld(t);
  EulerConverter::MatrixSXd b_R_w =
      base_angular_.GetRotationMatrixBaseToWorld(t).transpose();
  EulerConverter::MatrixSXd w_R_ee =
      ee_motion_angular_.GetRotationMatrixBaseToWorld(t);
  EulerConverter::MatrixSXd ee_R_w =
      ee_motion_angular_.GetRotationMatrixBaseToWorld(t).transpose();

  Vector3d vector_base_to_ee_W = pos_ee_W - base_W;
  Vector3d vector_base_to_ee_B = b_R_w * (vector_base_to_ee_W);

  g.middleRows(GetRow(k, X), k3D) = vector_base_to_ee_B; // X, Y, Z

  Eigen::MatrixXd frnt_mtx =
      (S2__.transpose() * ee_R_w * w_R_b * S1__.transpose() * local_x__);
  Eigen::MatrixXd bck_mtx = (S2__.transpose() * ee_R_w * w_R_b *
                             S1__.transpose() * S1__ * b_R_w * w_R_ee * S2__);
  double frnt = frnt_mtx(0, 0);
  double bck = bck_mtx(0, 0);

  Eigen::VectorXd val_rot(1);
  val_rot << frnt * std::pow(bck, -0.5);
  g.middleRows(GetRow(k, k3D), 1) = val_rot; // cos(yaw)
}

void RangeOfMotionBoxConstraint::UpdateBoundsAtInstance(
    double t, int k, VecBound &bounds) const {
  // X, Y, Z
  for (int dim = 0; dim < k3D; ++dim) {
    ifopt::Bounds b;
    b += nominal_ee_pos_B_(dim);
    b.upper_ += max_deviation_from_nominal_(dim);
    // b.lower_ -= max_deviation_from_nominal_(dim);
    b.lower_ += min_deviation_from_nominal_(dim);
    bounds.at(GetRow(k, dim)) = b;
  }
  // Rot
  ifopt::Bounds b;
  b.upper_ = max_cos_;
  b.lower_ = min_cos_;
  bounds.at(GetRow(k, k3D)) = b;
}

void RangeOfMotionBoxConstraint::UpdateJacobianAtInstance(double t, int k,
                                                          std::string var_set,
                                                          Jacobian &jac) const {

  EulerConverter::MatrixSXd w_R_b =
      base_angular_.GetRotationMatrixBaseToWorld(t);
  EulerConverter::MatrixSXd b_R_w =
      base_angular_.GetRotationMatrixBaseToWorld(t).transpose();
  EulerConverter::MatrixSXd w_R_ee =
      ee_motion_angular_.GetRotationMatrixBaseToWorld(t);
  EulerConverter::MatrixSXd ee_R_w =
      ee_motion_angular_.GetRotationMatrixBaseToWorld(t).transpose();

  int row_start = GetRow(k, X);

  if (var_set == id::base_lin_nodes) {
    jac.middleRows(row_start, k3D) =
        -1 * b_R_w * base_linear_->GetJacobianWrtNodes(t, kPos);
  }

  if (var_set == id::base_ang_nodes) {
    // X, Y, Z
    Vector3d base_W = base_linear_->GetPoint(t).p();
    Vector3d ee_pos_W = ee_motion_linear_->GetPoint(t).p();
    Vector3d r_W = ee_pos_W - base_W;

    jac.middleRows(row_start, k3D) =
        base_angular_.DerivOfRotVecMult(t, r_W, true);

    // Rot
    Eigen::MatrixXd frnt_mtx =
        (S2__.transpose() * ee_R_w * w_R_b * S1__.transpose() * local_x__);
    Eigen::MatrixXd bck_mtx = (S2__.transpose() * ee_R_w * w_R_b *
                               S1__.transpose() * S1__ * b_R_w * w_R_ee * S2__);
    double frnt = frnt_mtx(0, 0);
    double bck = bck_mtx(0, 0);

    Jacobian jac_rot, jac1, jac2, jac3;
    Eigen::Vector3d v1 = S1_.transpose() * local_x_;
    Eigen::Vector3d v2 = S1_.transpose() * S1_ * b_R_w * w_R_ee * S2_;
    Eigen::Vector3d v3 = w_R_ee * S2_;

    jac1 = S2__.transpose() * ee_R_w *
           base_angular_.DerivOfRotVecMult(t, v1, false) * std::pow(bck, -0.5);
    jac2 = S2__.transpose() * ee_R_w *
           base_angular_.DerivOfRotVecMult(t, v2, false);
    jac3 = S2__.transpose() * ee_R_w * w_R_b * S1__.transpose() * S1__ *
           base_angular_.DerivOfRotVecMult(t, v3, true);
    jac_rot = jac1 - 0.5 * frnt * std::pow(bck, -1.5) * (jac2 + jac3);

    jac.middleRows(row_start + k3D, 1) = jac_rot;
  }

  if (var_set == id::EEMotionLinNodes(ee_)) {
    jac.middleRows(row_start, k3D) =
        b_R_w * ee_motion_linear_->GetJacobianWrtNodes(t, kPos);
  }

  if (var_set == id::EEMotionAngNodes(ee_)) {

    Eigen::MatrixXd frnt_mtx =
        (S2__.transpose() * ee_R_w * w_R_b * S1__.transpose() * local_x__);
    Eigen::MatrixXd bck_mtx = (S2__.transpose() * ee_R_w * w_R_b *
                               S1__.transpose() * S1__ * b_R_w * w_R_ee * S2__);
    double frnt = frnt_mtx(0, 0);
    double bck = bck_mtx(0, 0);

    Jacobian jac_rot, jac1, jac2, jac3;
    Eigen::Vector3d v1 = w_R_b * S1_.transpose() * local_x_;
    Eigen::Vector3d v2 = w_R_b * S1_.transpose() * S1_ * b_R_w * w_R_ee * S2_;
    Eigen::Vector3d v3 = S2_;

    jac1 = S2__.transpose() *
           ee_motion_angular_.DerivOfRotVecMult(t, v1, true) *
           std::pow(bck, -0.5);
    jac2 = S2__.transpose() * ee_motion_angular_.DerivOfRotVecMult(t, v2, true);
    jac3 = S2__.transpose() * ee_R_w * w_R_b * S1__.transpose() * S1__ * b_R_w *
           ee_motion_angular_.DerivOfRotVecMult(t, v3, false);
    jac_rot = jac1 - 0.5 * frnt * std::pow(bck, -1.5) * (jac2 + jac3);

    jac.middleRows(row_start + k3D, 1) = jac_rot;
  }

  if (var_set == id::EESchedule(ee_)) {

    jac.middleRows(row_start, k3D) =
        b_R_w * ee_motion_linear_->GetJacobianOfPosWrtDurations(t);

    Eigen::MatrixXd frnt_mtx =
        (S2__.transpose() * ee_R_w * w_R_b * S1__.transpose() * local_x__);
    Eigen::MatrixXd bck_mtx = (S2__.transpose() * ee_R_w * w_R_b *
                               S1__.transpose() * S1__ * b_R_w * w_R_ee * S2__);
    double frnt = frnt_mtx(0, 0);
    double bck = bck_mtx(0, 0);

    Jacobian jac_rot, jac1, jac2, jac3;
    Eigen::Vector3d v1 = w_R_b * S1_.transpose() * local_x_;
    Eigen::Vector3d v2 = w_R_b * S1_.transpose() * S1_ * b_R_w * w_R_ee * S2_;
    Eigen::Vector3d v3 = S2_;

    jac1 =
        S2__.transpose() *
        ee_motion_angular_.DerivOfRotVecMultWrtScheduleVariables(t, v1, true) *
        std::pow(bck, -0.5);
    jac2 =
        S2__.transpose() *
        ee_motion_angular_.DerivOfRotVecMultWrtScheduleVariables(t, v2, true);
    jac3 =
        S2__.transpose() * ee_R_w * w_R_b * S1__.transpose() * S1__ * b_R_w *
        ee_motion_angular_.DerivOfRotVecMultWrtScheduleVariables(t, v3, false);
    jac_rot = jac1 - 0.5 * frnt * std::pow(bck, -1.5) * (jac2 + jac3);

    jac.middleRows(row_start + k3D, 1) = jac_rot;
  }
}

} // namespace towr_plus
