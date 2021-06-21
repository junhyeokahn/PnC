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

#include <towr_plus/constraints/terrain_constraint.h>
#include <util/util.hpp>

#include <iostream>

namespace towr_plus {

TerrainConstraint::TerrainConstraint(const HeightMap::Ptr &terrain, int id,
                                     std::string ee_motion_lin,
                                     std::string ee_motion_ang)
    : ConstraintSet(kSpecifyLater, "terrain-" + std::to_string(id)) {
  ee_motion_lin_id_ = ee_motion_lin;
  ee_motion_ang_id_ = ee_motion_ang;
  terrain_ = terrain;
}

void TerrainConstraint::InitVariableDependedQuantities(const VariablesPtr &x) {
  ee_motion_lin_ = x->GetComponent<NodesVariablesPhaseBased>(ee_motion_lin_id_);
  ee_motion_ang_ = x->GetComponent<NodesVariablesPhaseBased>(ee_motion_ang_id_);

  // skip first node, b/c already constrained by initial stance
  for (int id = 1; id < ee_motion_lin_->GetNodes().size(); ++id) {
    lin_node_ids_.push_back(id);
  }

  // skip first node, b/c already constrained by initial stance
  for (int id = 1; id < ee_motion_ang_->GetNodes().size(); ++id) {
    if (ee_motion_ang_->IsConstantNode(id)) {
      contact_nodes_ids_.push_back(id);
    }
  }

  n_lin_ = lin_node_ids_.size();
  n_ang_ = 3 * contact_nodes_ids_.size();

  SetRows(n_lin_ + n_ang_);
}

Eigen::VectorXd TerrainConstraint::GetValues() const {
  VectorXd g(GetRows());

  auto lin_nodes = ee_motion_lin_->GetNodes();
  auto ang_nodes = ee_motion_ang_->GetNodes();
  int row = 0;
  for (int id : lin_node_ids_) {
    Vector3d p = lin_nodes.at(id).p();
    g(row++) = p.z() - terrain_->GetHeight(p.x(), p.y());
  }

  for (int id : contact_nodes_ids_) {
    Vector3d lin_p = lin_nodes.at(id).p();
    Vector3d normal_vector =
        terrain_->GetNormalizedBasis(HeightMap::Normal, lin_p.x(), lin_p.y());
    Vector3d ang_p = ang_nodes.at(id).p();
    Eigen::Matrix3d rot = euler_xyz_to_rot(ang_p);
    for (int i = 0; i < 3; ++i) {
      g(row++) = rot(i, 2) - normal_vector(i);
    }
  }

  return g;
}

TerrainConstraint::VecBound TerrainConstraint::GetBounds() const {
  VecBound bounds(GetRows());
  double max_distance_above_terrain = 1e20; // [m]

  int row = 0;
  // Foot Height
  for (int id : lin_node_ids_) {
    if (ee_motion_lin_->IsConstantNode(id)) {
      bounds.at(row) = ifopt::BoundZero;
    } else {
      bounds.at(row) = ifopt::Bounds(0.0, max_distance_above_terrain);
    }
    row++;
  }

  // Foot Orientation
  for (int id : contact_nodes_ids_) {
    for (int i = 0; i < 3; ++i) {
      bounds.at(row++) = ifopt::BoundZero;
      // bounds.at(row++) = ifopt::Bounds(-0.05, 0.05);
    }
  }

  return bounds;
}

void TerrainConstraint::FillJacobianBlock(std::string var_set,
                                          Jacobian &jac) const {
  if (var_set == ee_motion_lin_->GetName()) {
    auto lin_nodes = ee_motion_lin_->GetNodes();
    int row = 0;
    // Height Constraints
    for (int id : lin_node_ids_) {
      int idx = ee_motion_lin_->GetOptIndex(
          NodesVariables::NodeValueInfo(id, kPos, Z));
      jac.coeffRef(row, idx) = 1.0;

      Vector3d p = lin_nodes.at(id).p();
      for (auto dim : {X, Y}) {
        int idx = ee_motion_lin_->GetOptIndex(
            NodesVariables::NodeValueInfo(id, kPos, dim));
        jac.coeffRef(row, idx) =
            -terrain_->GetDerivativeOfHeightWrt(To2D(dim), p.x(), p.y());
      }
      row++;
    }

    // Orientation Constraints
    for (int id : contact_nodes_ids_) {
      Vector3d lin_p = lin_nodes.at(id).p();
      Vector3d round_n_round_x = terrain_->GetDerivativeOfNormalizedBasisWrt(
          HeightMap::Normal, X_, lin_p.x(), lin_p.y());
      Vector3d round_n_round_y = terrain_->GetDerivativeOfNormalizedBasisWrt(
          HeightMap::Normal, Y_, lin_p.x(), lin_p.y());
      int idx_x = ee_motion_lin_->GetOptIndex(
          NodesVariables::NodeValueInfo(id, kPos, X));
      int idx_y = ee_motion_lin_->GetOptIndex(
          NodesVariables::NodeValueInfo(id, kPos, Y));

      jac.coeffRef(row, idx_x) = -round_n_round_x(0);
      jac.coeffRef(row++, idx_y) = -round_n_round_y(0);

      jac.coeffRef(row, idx_x) = -round_n_round_x(1);
      jac.coeffRef(row++, idx_y) = -round_n_round_y(1);

      jac.coeffRef(row, idx_x) = -round_n_round_x(2);
      jac.coeffRef(row++, idx_y) = -round_n_round_y(2);
    }
  }

  if (var_set == ee_motion_ang_->GetName()) {
    int row = n_lin_;
    for (int id : contact_nodes_ids_) {
      auto ang_nodes = ee_motion_ang_->GetNodes();
      int idx_x = ee_motion_ang_->GetOptIndex(
          NodesVariables::NodeValueInfo(id, kPos, X));
      int idx_y = ee_motion_ang_->GetOptIndex(
          NodesVariables::NodeValueInfo(id, kPos, Y));
      int idx_z = ee_motion_ang_->GetOptIndex(
          NodesVariables::NodeValueInfo(id, kPos, Z));
      Vector3d ang_p = ang_nodes.at(id).p();
      double x = ang_p(0);
      double y = ang_p(1);
      double z = ang_p(2);

      jac.coeffRef(row, idx_x) = cos(x) * sin(z) - sin(x) * cos(z) * sin(y);
      jac.coeffRef(row, idx_y) = cos(x) * cos(z) * cos(y);
      jac.coeffRef(row++, idx_z) = sin(x) * cos(z) - cos(x) * sin(z) * sin(y);

      jac.coeffRef(row, idx_x) = -sin(x) * sin(y) * sin(z) - cos(z) * cos(x);
      jac.coeffRef(row, idx_y) = cos(x) * cos(y) * sin(z);
      jac.coeffRef(row++, idx_z) = cos(x) * sin(y) * cos(z) + sin(z) * sin(x);

      jac.coeffRef(row, idx_x) = -sin(x) * cos(y);
      jac.coeffRef(row++, idx_y) = -cos(x) * sin(y);
    }
  }
}

} /* namespace towr_plus */
