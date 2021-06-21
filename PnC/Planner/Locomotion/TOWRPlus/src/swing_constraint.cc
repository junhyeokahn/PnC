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
#include <towr_plus/constraints/swing_constraint.h>
#include <towr_plus/variables/cartesian_dimensions.h>

namespace towr_plus {

SwingConstraint::SwingConstraint(int ee, std::string ee_motion_linear_id,
                                 std::string ee_motion_angular_id,
                                 double t_swing_avg)
    : ConstraintSet(kSpecifyLater, "swing-" + std::to_string(ee)) {
  ee_motion_linear_id_ = ee_motion_linear_id;
  ee_motion_angular_id_ = ee_motion_angular_id;
  t_swing_avg_ = t_swing_avg;
}

void towr_plus::SwingConstraint::InitVariableDependedQuantities(
    const VariablesPtr &x) {
  ee_motion_linear_ =
      x->GetComponent<NodesVariablesPhaseBased>(ee_motion_linear_id_);
  ee_motion_angular_ =
      x->GetComponent<NodesVariablesPhaseBased>(ee_motion_angular_id_);

  pure_swing_node_ids_ = ee_motion_linear_->GetIndicesOfNonConstantNodes();

  // constrain xy position and velocity of every swing linear node
  lin_constraint_count_ =
      pure_swing_node_ids_.size() * Node::n_derivatives * k2D;
  // constrain xyz position and velocity of every swing angular node
  ang_constraint_count_ =
      pure_swing_node_ids_.size() * Node::n_derivatives * k3D;

  SetRows(lin_constraint_count_ + ang_constraint_count_);
}

Eigen::VectorXd SwingConstraint::GetValues() const {
  // x, xdot, y, ydot, x, xdot, y, ydot, z, zdot
  VectorXd g(GetRows());

  int row = 0;
  auto lin_nodes = ee_motion_linear_->GetNodes();
  for (int node_id : pure_swing_node_ids_) {
    // Linear Node
    // assumes two splines per swingphase and starting and ending in stance
    auto curr_lin_node = lin_nodes.at(node_id);

    Vector2d prev_lin = lin_nodes.at(node_id - 1).p().topRows<k2D>();
    Vector2d next_lin = lin_nodes.at(node_id + 1).p().topRows<k2D>();

    Vector2d distance_xy = next_lin - prev_lin;
    Vector2d xy_center = prev_lin + 0.5 * distance_xy;
    Vector2d des_vel_center =
        distance_xy / t_swing_avg_; // linear interpolation not accurate
    for (auto dim : {X, Y}) {
      g(row++) = curr_lin_node.p()(dim) - xy_center(dim);
      g(row++) = curr_lin_node.v()(dim) - des_vel_center(dim);
    }
  }

  auto ang_nodes = ee_motion_angular_->GetNodes();
  for (int node_id : pure_swing_node_ids_) {
    // Angular Node
    // assumes two splines per swingphase and starting and ending in stance

    auto curr_ang_node = ang_nodes.at(node_id);

    Eigen::Vector3d prev_ang = ang_nodes.at(node_id - 1).p();
    Eigen::Vector3d next_ang = ang_nodes.at(node_id + 1).p();
    Eigen::Vector3d distance_xyz = next_ang - prev_ang;
    Eigen::Vector3d xyz_center = prev_ang + 0.5 * distance_xyz;
    Eigen::Vector3d des_vel_center =
        distance_xyz / t_swing_avg_; // linear interpolation not accurate
    for (auto dim : {X, Y, Z}) {
      g(row++) = curr_ang_node.p()(dim) - xyz_center(dim);
      g(row++) = curr_ang_node.v()(dim) - des_vel_center(dim);
    }
  }

  return g;
}

SwingConstraint::VecBound SwingConstraint::GetBounds() const {
  return VecBound(GetRows(), ifopt::BoundZero);
}

void SwingConstraint::FillJacobianBlock(std::string var_set,
                                        Jacobian &jac) const {
  if (var_set == ee_motion_linear_->GetName()) {
    int row = 0;
    for (int node_id : pure_swing_node_ids_) {
      for (auto dim : {X, Y}) {
        // position constraint
        jac.coeffRef(row,
                     ee_motion_linear_->GetOptIndex(
                         NodesVariables::NodeValueInfo(node_id, kPos, dim))) =
            1.0; // current node
        jac.coeffRef(
            row, ee_motion_linear_->GetOptIndex(NodesVariables::NodeValueInfo(
                     node_id + 1, kPos, dim))) = -0.5; // next node
        jac.coeffRef(
            row, ee_motion_linear_->GetOptIndex(NodesVariables::NodeValueInfo(
                     node_id - 1, kPos, dim))) = -0.5; // previous node
        row++;

        // velocity constraint
        jac.coeffRef(row,
                     ee_motion_linear_->GetOptIndex(
                         NodesVariables::NodeValueInfo(node_id, kVel, dim))) =
            1.0; // current node
        jac.coeffRef(
            row, ee_motion_linear_->GetOptIndex(
                     NodesVariables::NodeValueInfo(node_id + 1, kPos, dim))) =
            -1.0 / t_swing_avg_; // next node
        jac.coeffRef(
            row, ee_motion_linear_->GetOptIndex(
                     NodesVariables::NodeValueInfo(node_id - 1, kPos, dim))) =
            +1.0 / t_swing_avg_; // previous node
        row++;
      }
    }
  }

  if (var_set == ee_motion_angular_->GetName()) {

    int row = lin_constraint_count_;
    for (int node_id : pure_swing_node_ids_) {
      for (auto dim : {X, Y, Z}) {
        // position constraint
        jac.coeffRef(row,
                     ee_motion_angular_->GetOptIndex(
                         NodesVariables::NodeValueInfo(node_id, kPos, dim))) =
            1.0; // current node
        jac.coeffRef(
            row, ee_motion_angular_->GetOptIndex(NodesVariables::NodeValueInfo(
                     node_id + 1, kPos, dim))) = -0.5; // next node
        jac.coeffRef(
            row, ee_motion_angular_->GetOptIndex(NodesVariables::NodeValueInfo(
                     node_id - 1, kPos, dim))) = -0.5; // previous node
        row++;

        // velocity constraint
        jac.coeffRef(row,
                     ee_motion_angular_->GetOptIndex(
                         NodesVariables::NodeValueInfo(node_id, kVel, dim))) =
            1.0; // current node
        jac.coeffRef(
            row, ee_motion_angular_->GetOptIndex(
                     NodesVariables::NodeValueInfo(node_id + 1, kPos, dim))) =
            -1.0 / t_swing_avg_; // next node
        jac.coeffRef(
            row, ee_motion_angular_->GetOptIndex(
                     NodesVariables::NodeValueInfo(node_id - 1, kPos, dim))) =
            +1.0 / t_swing_avg_; // previous node
        row++;
      }
    }
  }
}

} /* namespace towr_plus */
