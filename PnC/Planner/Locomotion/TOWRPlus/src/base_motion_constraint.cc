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

#include <towr_plus/constraints/base_motion_constraint.h>
#include <towr_plus/variables/cartesian_dimensions.h>
#include <towr_plus/variables/spline_holder.h>
#include <towr_plus/variables/variable_names.h>

namespace towr_plus {

BaseMotionConstraint::BaseMotionConstraint(double T, double dt,
                                           const SplineHolder &spline_holder,
                                           double min_height, double max_height)
    : TimeDiscretizationConstraint(T, dt, "baseMotion") {
  base_linear_ = spline_holder.base_linear_;
  base_angular_ = spline_holder.base_angular_;

  double dev_rad = 0.1;
  node_bounds_.resize(k6D);
  node_bounds_.at(AX) = Bounds(-dev_rad, dev_rad);
  node_bounds_.at(AY) = Bounds(-dev_rad, dev_rad);
  node_bounds_.at(AZ) = ifopt::NoBound;

  double z_init = base_linear_->GetPoint(0.0).p().z();
  node_bounds_.at(LX) = ifopt::NoBound;
  node_bounds_.at(LY) = ifopt::NoBound;
  node_bounds_.at(LZ) = Bounds(min_height, max_height);

  int n_constraints_per_node = node_bounds_.size();
  SetRows(GetNumberOfNodes() * n_constraints_per_node);
}

void BaseMotionConstraint::UpdateConstraintAtInstance(double t, int k,
                                                      VectorXd &g) const {
  g.middleRows(GetRow(k, LX), k3D) = base_linear_->GetPoint(t).p();
  g.middleRows(GetRow(k, AX), k3D) = base_angular_->GetPoint(t).p();
}

void BaseMotionConstraint::UpdateBoundsAtInstance(double t, int k,
                                                  VecBound &bounds) const {
  for (int dim = 0; dim < node_bounds_.size(); ++dim)
    bounds.at(GetRow(k, dim)) = node_bounds_.at(dim);
}

void BaseMotionConstraint::UpdateJacobianAtInstance(double t, int k,
                                                    std::string var_set,
                                                    Jacobian &jac) const {
  if (var_set == id::base_ang_nodes)
    jac.middleRows(GetRow(k, AX), k3D) =
        base_angular_->GetJacobianWrtNodes(t, kPos);

  if (var_set == id::base_lin_nodes)
    jac.middleRows(GetRow(k, LX), k3D) =
        base_linear_->GetJacobianWrtNodes(t, kPos);
}

int BaseMotionConstraint::GetRow(int node, int dim) const {
  return node * node_bounds_.size() + dim;
}

} /* namespace towr_plus */
