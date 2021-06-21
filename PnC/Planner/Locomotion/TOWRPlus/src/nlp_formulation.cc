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

#include <fstream>
#include <iostream>

#include <configuration.h>
#include <dubins.hpp>

#include <towr_plus/constraints/base_motion_constraint.h>
#include <towr_plus/constraints/dynamic_constraint.h>
#include <towr_plus/constraints/force_constraint.h>
#include <towr_plus/constraints/range_of_motion_box_constraint.h>
#include <towr_plus/constraints/range_of_motion_circle_constraint.h>
#include <towr_plus/constraints/spline_acc_constraint.h>
#include <towr_plus/constraints/swing_constraint.h>
#include <towr_plus/constraints/terrain_constraint.h>
#include <towr_plus/constraints/total_duration_constraint.h>
#include <towr_plus/costs/final_node_cost.h>
#include <towr_plus/costs/intermediate_node_cost.h>
#include <towr_plus/costs/node_cost.h>
#include <towr_plus/costs/node_difference_cost.h>
#include <towr_plus/nlp_formulation.h>
#include <towr_plus/variables/nodes_variables_all.h>
#include <towr_plus/variables/phase_durations.h>
#include <towr_plus/variables/variable_names.h>

namespace towr_plus {
NlpFormulation::NlpFormulation() {
  using namespace std;
  cout << "\n";
  cout << "********************************************************************"
          "**********\n";
  cout << "                                    TOWR+ \n";
  cout << "                               \u00a9 Junhyeok Ahn \n";
  cout << "********************************************************************"
          "**********";
  cout << "\n\n";

  rom_buf = 0.025;

  one_hot_ee_motion_lin_.resize(2);
  one_hot_ee_motion_ang_.resize(2);
  one_hot_ee_wrench_lin_.resize(2);
  one_hot_ee_wrench_ang_.resize(2);
  b_initialize_ = false;
}

NlpFormulation::VariablePtrVec
NlpFormulation::GetVariableSets(SplineHolder &spline_holder) {
  VariablePtrVec vars;

  auto base_motion = MakeBaseVariables();
  vars.insert(vars.end(), base_motion.begin(), base_motion.end());

  auto ee_motion_lin = MakeEEMotionLinVariables();
  vars.insert(vars.end(), ee_motion_lin.begin(), ee_motion_lin.end());

  auto ee_motion_ang = MakeEEMotionAngVariables();
  vars.insert(vars.end(), ee_motion_ang.begin(), ee_motion_ang.end());

  auto ee_wrench_lin = MakeWrenchLinVariables();
  vars.insert(vars.end(), ee_wrench_lin.begin(), ee_wrench_lin.end());

  auto ee_wrench_ang = MakeWrenchAngVariables();
  vars.insert(vars.end(), ee_wrench_ang.begin(), ee_wrench_ang.end());

  auto contact_schedule = MakeContactScheduleVariables();
  // can also just be fixed timings that aren't optimized over, but still
  // added to spline_holder.
  if (params_.IsOptimizeTimings()) {
    vars.insert(vars.end(), contact_schedule.begin(), contact_schedule.end());
  }

  // stores these readily constructed spline
  spline_holder = SplineHolder(base_motion.at(0), // linear
                               base_motion.at(1), // angular
                               params_.GetBasePolyDurations(),
                               ee_motion_lin, // linear
                               ee_motion_ang, // angular
                               ee_wrench_lin, // linear
                               ee_wrench_ang, // angular
                               contact_schedule, params_.IsOptimizeTimings());

  return vars;
}

std::vector<NodesVariables::Ptr> NlpFormulation::MakeBaseVariables() const {
  std::vector<NodesVariables::Ptr> vars;

  int n_nodes = params_.GetBasePolyDurations().size() + 1;

  auto spline_lin =
      std::make_shared<NodesVariablesAll>(n_nodes, k3D, id::base_lin_nodes);

  double x = final_base_.lin.p().x();
  double y = final_base_.lin.p().y();
  double z = terrain_->GetHeight(x, y) -
             model_.kinematic_model_->GetNominalStanceInBase().front().z();
  Vector3d final_pos(x, y, z);

  if (b_initialize_) {
    spline_lin->SetVariables(one_hot_base_lin_);
  } else {
    spline_lin->SetByLinearInterpolation(initial_base_.lin.p(), final_pos,
                                         params_.GetTotalTime());
  }
  spline_lin->AddStartBound(kPos, {X, Y, Z}, initial_base_.lin.p());
  spline_lin->AddStartBound(kVel, {X, Y, Z}, initial_base_.lin.v());
  spline_lin->AddFinalBound(kPos, {X, Y, Z}, final_pos);
  spline_lin->AddFinalBound(kVel, {X, Y, Z}, final_base_.lin.v());
  vars.push_back(spline_lin);

  auto spline_ang =
      std::make_shared<NodesVariablesAll>(n_nodes, k3D, id::base_ang_nodes);
  if (b_initialize_) {
    spline_ang->SetVariables(one_hot_base_ang_);
  } else {
    spline_ang->SetByLinearInterpolation(
        initial_base_.ang.p(), final_base_.ang.p(), params_.GetTotalTime());
  }
  spline_ang->AddStartBound(kPos, {X, Y, Z}, initial_base_.ang.p());
  spline_ang->AddStartBound(kVel, {X, Y, Z}, initial_base_.ang.v());
  spline_ang->AddFinalBound(kPos, {X, Y, Z}, final_base_.ang.p());
  spline_ang->AddFinalBound(kVel, {X, Y, Z}, final_base_.ang.v());
  vars.push_back(spline_ang);

  return vars;
}

std::vector<NodesVariablesPhaseBased::Ptr>
NlpFormulation::MakeEEMotionLinVariables() const {
  std::vector<NodesVariablesPhaseBased::Ptr> vars;

  double T = params_.GetTotalTime();
  for (int ee = 0; ee < params_.GetEECount(); ee++) {
    auto lin_nodes = std::make_shared<NodesVariablesEEMotion>(
        params_.GetPhaseCount(ee), params_.ee_in_contact_at_start_.at(ee),
        id::EEMotionLinNodes(ee), params_.ee_polynomials_per_swing_phase_,
        true);

    // initialize towards final footholds
    Eigen::Vector3d final_base_euler(final_base_.ang.p().x(),
                                     final_base_.ang.p().y(),
                                     final_base_.ang.p().z());
    Eigen::Matrix3d w_R_b =
        EulerConverter::GetRotationMatrixBaseToWorld(final_base_euler);
    double com_x = final_base_.lin.p().x();
    double com_y = final_base_.lin.p().y();
    double com_z = final_base_.lin.p().z();
    Eigen::Vector3d final_base_pos(com_x, com_y,
                                   terrain_->GetHeight(com_x, com_y) + com_z);
    Vector3d final_ee_motion_lin =
        final_base_pos +
        w_R_b * model_.kinematic_model_->GetNominalStanceInBase().at(ee);
    double x = final_ee_motion_lin.x();
    double y = final_ee_motion_lin.y();
    double z = terrain_->GetHeight(x, y);

    if (b_initialize_) {
      lin_nodes->SetVariables(one_hot_ee_motion_lin_.at(ee));
    } else {
      lin_nodes->SetByLinearInterpolation(initial_ee_motion_lin_.at(ee),
                                          Vector3d(x, y, z), T);
    }

    Eigen::VectorXd zero3 = Eigen::VectorXd::Zero(3);
    lin_nodes->AddStartBound(kPos, {X, Y, Z}, initial_ee_motion_lin_.at(ee));
    lin_nodes->AddStartBound(kVel, {X, Y, Z}, zero3);
    lin_nodes->AddFinalBound(kPos, {X, Y, Z}, Vector3d(x, y, z));
    lin_nodes->AddFinalBound(kVel, {X, Y, Z}, zero3);
    vars.push_back(lin_nodes);
  }

  return vars;
}

std::vector<NodesVariablesPhaseBased::Ptr>
NlpFormulation::MakeEEMotionAngVariables() const {
  std::vector<NodesVariablesPhaseBased::Ptr> vars;

  for (int ee = 0; ee < params_.GetEECount(); ee++) {
    auto ang_nodes = std::make_shared<NodesVariablesEEMotion>(
        params_.GetPhaseCount(ee), params_.ee_in_contact_at_start_.at(ee),
        id::EEMotionAngNodes(ee), params_.ee_polynomials_per_swing_phase_,
        false);

    // compute final footholds orientation
    Eigen::Vector3d base_euler_xyz(final_base_.ang.p().x(),
                                   final_base_.ang.p().y(),
                                   final_base_.ang.p().z());
    Eigen::Matrix3d base_rot = euler_xyz_to_rot(base_euler_xyz);
    double com_x = final_base_.lin.p().x();
    double com_y = final_base_.lin.p().y();
    double com_z = final_base_.lin.p().z();
    Eigen::Vector3d final_base_pos(com_x, com_y,
                                   terrain_->GetHeight(com_x, com_y) + com_z);
    Vector3d final_ee_motion_lin =
        final_base_pos +
        base_rot * model_.kinematic_model_->GetNominalStanceInBase().at(ee);
    double x = final_ee_motion_lin(0);
    double y = final_ee_motion_lin(1);
    Eigen::Vector3d foot_x_axis =
        terrain_->GetProjectionToTerrain(x, y, base_rot.col(X), true);
    Eigen::Vector3d foot_z_axis =
        terrain_->GetNormalizedBasis(HeightMap::Normal, x, y);
    Eigen::Vector3d foot_y_axis = foot_z_axis.cross(foot_x_axis);
    Eigen::Matrix3d foot_rot;
    foot_rot.col(0) = foot_x_axis;
    foot_rot.col(1) = foot_y_axis;
    foot_rot.col(2) = foot_z_axis;
    Eigen::Vector3d final_foot_euler_angles;
    // Little detail: Eigen returns zyx order
    final_foot_euler_angles << foot_rot.eulerAngles(2, 1, 0)(2),
        foot_rot.eulerAngles(2, 1, 0)(1), foot_rot.eulerAngles(2, 1, 0)(0);
    for (int i = 0; i < 3; ++i) {
      if (final_foot_euler_angles[i] >= M_PI / 2.)
        final_foot_euler_angles[i] -= M_PI;
      if (final_foot_euler_angles[i] <= -M_PI / 2.)
        final_foot_euler_angles[i] += M_PI;
    }

    if (b_initialize_) {
      ang_nodes->SetVariables(one_hot_ee_motion_ang_.at(ee));
    } else {
      ang_nodes->SetByLinearInterpolation(initial_ee_motion_ang_.at(ee),
                                          final_foot_euler_angles,
                                          params_.GetTotalTime());
    }
    Eigen::VectorXd zero3 = Eigen::VectorXd::Zero(3);
    ang_nodes->AddStartBound(kPos, {X, Y, Z}, initial_ee_motion_ang_.at(ee));
    ang_nodes->AddStartBound(kVel, {X, Y, Z}, zero3);
    ang_nodes->AddFinalBound(kPos, {X, Y, Z}, final_foot_euler_angles);
    ang_nodes->AddFinalBound(kVel, {X, Y, Z}, zero3);
    vars.push_back(ang_nodes);
  }
  return vars;
}

std::vector<NodesVariablesPhaseBased::Ptr>
NlpFormulation::MakeWrenchLinVariables() const {
  std::vector<NodesVariablesPhaseBased::Ptr> vars;
  double T = params_.GetTotalTime();
  for (int ee = 0; ee < params_.GetEECount(); ee++) {
    auto lin_nodes = std::make_shared<NodesVariablesEEForce>(
        params_.GetPhaseCount(ee), params_.ee_in_contact_at_start_.at(ee),
        id::EEWrenchLinNodes(ee), params_.force_polynomials_per_stance_phase_);

    double m = model_.dynamic_model_->m();
    double g = model_.dynamic_model_->g();

    Vector3d f_stance(0.0, 0.0, m * g / params_.GetEECount());
    if (b_initialize_) {
      lin_nodes->SetVariables(one_hot_ee_wrench_lin_.at(ee));
    } else {
      lin_nodes->SetByLinearInterpolation(f_stance, f_stance,
                                          T); // stay constant
    }
    vars.push_back(lin_nodes);
  }
  return vars;
}

std::vector<NodesVariablesPhaseBased::Ptr>
NlpFormulation::MakeWrenchAngVariables() const {
  std::vector<NodesVariablesPhaseBased::Ptr> vars;
  for (int ee = 0; ee < params_.GetEECount(); ++ee) {
    auto ang_nodes = std::make_shared<NodesVariablesEEForce>(
        params_.GetPhaseCount(ee), params_.ee_in_contact_at_start_.at(ee),
        id::EEWrenchAngNodes(ee), params_.force_polynomials_per_stance_phase_);
    if (b_initialize_) {
      ang_nodes->SetVariables(one_hot_ee_wrench_ang_.at(ee));
    }
    vars.push_back(ang_nodes);
  }
  return vars;
}

std::vector<PhaseDurations::Ptr>
NlpFormulation::MakeContactScheduleVariables() const {
  std::vector<PhaseDurations::Ptr> vars;

  for (int ee = 0; ee < params_.GetEECount(); ee++) {
    auto var =
        std::make_shared<PhaseDurations>(ee, params_.ee_phase_durations_.at(ee),
                                         params_.ee_in_contact_at_start_.at(ee),
                                         params_.bound_phase_duration_.first,
                                         params_.bound_phase_duration_.second);
    vars.push_back(var);
  }

  return vars;
}

NlpFormulation::ContraintPtrVec
NlpFormulation::GetConstraints(const SplineHolder &spline_holder) const {
  ContraintPtrVec constraints;
  for (auto name : params_.constraints_) {
    for (auto c : GetConstraint(name, spline_holder)) {
      constraints.push_back(c);
    }
  }

  return constraints;
}

NlpFormulation::ContraintPtrVec
NlpFormulation::GetConstraint(Parameters::ConstraintName name,
                              const SplineHolder &s) const {
  switch (name) {
  case Parameters::Dynamic:
    return MakeDynamicConstraint(s);
  case Parameters::EndeffectorRom:
    return MakeRangeOfMotionBoxConstraint(s);
  case Parameters::BaseRom:
    return MakeBaseRangeOfMotionConstraint(s);
  case Parameters::TotalTime:
    return MakeTotalTimeConstraint();
  case Parameters::Terrain:
    return MakeTerrainConstraint();
  case Parameters::Force:
    return MakeForceConstraint(s);
  case Parameters::Swing:
    return MakeSwingConstraint();
  case Parameters::BaseAcc:
    return MakeBaseAccConstraint(s);
  default:
    throw std::runtime_error("constraint not defined!");
  }
}

NlpFormulation::ContraintPtrVec
NlpFormulation::MakeBaseRangeOfMotionConstraint(const SplineHolder &s) const {
  // Figure out min and max height of com
  double min_z(initial_base_.lin.p()(2));
  double max_z(initial_base_.lin.p()(2));
  double nominal_height(initial_base_.lin.p()(2));
  int n_check(100);
  double init_x = initial_base_.lin.p()(0);
  double init_y = initial_base_.lin.p()(1);
  double fin_x = final_base_.lin.p()(0);
  double fin_y = final_base_.lin.p()(1);
  for (int i = 0; i < n_check; ++i) {
    double x_check = init_x + i * (fin_x - init_x) / n_check;
    double y_check = init_y + i * (fin_y - init_y) / n_check;
    double nominal_z = terrain_->GetHeight(x_check, y_check) + nominal_height;
    min_z = std::min(nominal_z, min_z);
    max_z = std::max(nominal_z, max_z);
  }
  return {std::make_shared<BaseMotionConstraint>(
      params_.GetTotalTime(), params_.dt_constraint_base_motion_, s,
      min_z - rom_buf, max_z + rom_buf)};
}

NlpFormulation::ContraintPtrVec
NlpFormulation::MakeDynamicConstraint(const SplineHolder &s) const {
  auto constraint = std::make_shared<DynamicConstraint>(
      model_.dynamic_model_, params_.GetTotalTime(),
      params_.dt_constraint_dynamic_, s);
  return {constraint};
}

NlpFormulation::ContraintPtrVec
NlpFormulation::MakeRangeOfMotionBoxConstraint(const SplineHolder &s) const {
  ContraintPtrVec c;

  for (int ee = 0; ee < params_.GetEECount(); ee++) {
    auto rom = std::make_shared<RangeOfMotionBoxConstraint>(
        model_.kinematic_model_, params_.GetTotalTime(),
        params_.dt_constraint_range_of_motion_, ee, s);
    c.push_back(rom);
  }

  return c;
}

NlpFormulation::ContraintPtrVec
NlpFormulation::MakeTotalTimeConstraint() const {
  ContraintPtrVec c;
  double T = params_.GetTotalTime();

  for (int ee = 0; ee < params_.GetEECount(); ee++) {
    auto duration_constraint = std::make_shared<TotalDurationConstraint>(T, ee);
    c.push_back(duration_constraint);
  }

  return c;
}

NlpFormulation::ContraintPtrVec NlpFormulation::MakeTerrainConstraint() const {
  ContraintPtrVec constraints;

  for (int ee = 0; ee < params_.GetEECount(); ee++) {
    auto c = std::make_shared<TerrainConstraint>(
        terrain_, ee, id::EEMotionLinNodes(ee), id::EEMotionAngNodes(ee));
    constraints.push_back(c);
  }

  return constraints;
}

NlpFormulation::ContraintPtrVec
NlpFormulation::MakeForceConstraint(const SplineHolder &s) const {
  ContraintPtrVec constraints;

  for (int ee = 0; ee < params_.GetEECount(); ee++) {
    auto c = std::make_shared<ForceConstraint>(
        terrain_, params_.force_limit_in_normal_direction_,
        model_.kinematic_model_->GetFootHalfLength(),
        model_.kinematic_model_->GetFootHalfWidth(), ee, s);
    constraints.push_back(c);
  }

  return constraints;
}

NlpFormulation::ContraintPtrVec NlpFormulation::MakeSwingConstraint() const {
  ContraintPtrVec constraints;

  for (int ee = 0; ee < params_.GetEECount(); ee++) {
    auto swing = std::make_shared<SwingConstraint>(
        ee, id::EEMotionLinNodes(ee), id::EEMotionAngNodes(ee),
        params_.ee_phase_durations_.at(ee)[1]);
    constraints.push_back(swing);
  }

  return constraints;
}

NlpFormulation::ContraintPtrVec
NlpFormulation::MakeBaseAccConstraint(const SplineHolder &s) const {
  ContraintPtrVec constraints;

  constraints.push_back(std::make_shared<SplineAccConstraint>(
      s.base_linear_, id::base_lin_nodes));

  constraints.push_back(std::make_shared<SplineAccConstraint>(
      s.base_angular_, id::base_ang_nodes));

  return constraints;
}

NlpFormulation::ContraintPtrVec NlpFormulation::GetCosts() const {
  ContraintPtrVec costs;
  for (const auto &pair : params_.costs_) {
    for (auto c : GetCost(pair.first, pair.second)) {
      costs.push_back(c);
    }
  }

  return costs;
}

NlpFormulation::CostPtrVec
NlpFormulation::GetCost(const Parameters::CostName &name,
                        const Eigen::VectorXd weight) const {
  switch (name) {
  case Parameters::FinalBaseLinPosCost:
    return MakeFinalBaseLinCost(Dx::kPos, weight);
  case Parameters::FinalBaseLinVelCost:
    return MakeFinalBaseLinCost(Dx::kVel, weight);
  case Parameters::FinalBaseAngPosCost:
    return MakeFinalBaseAngCost(Dx::kPos, weight);
  case Parameters::FinalBaseAngVelCost:
    return MakeFinalBaseAngCost(Dx::kVel, weight);
  case Parameters::FinalEEMotionLinPosCost:
    return MakeFinalEEMotionLinPosCost(weight);
  case Parameters::FinalEEMotionAngPosCost:
    return MakeFinalEEMotionAngPosCost(weight);
  case Parameters::IntermediateBaseLinVelCost:
    return MakeIntermediateBaseLinCost(Dx::kVel, weight);
  case Parameters::IntermediateBaseAngVelCost:
    return MakeIntermediateBaseAngCost(Dx::kVel, weight);
  case Parameters::BaseLinVelDiffCost:
    return MakeBaseLinVelDiffCost(weight);
  case Parameters::BaseAngVelDiffCost:
    return MakeBaseAngVelDiffCost(weight);
  case Parameters::WrenchLinPosCost:
    return MakeWrenchLinCost(Dx::kPos, weight);
  case Parameters::WrenchLinVelCost:
    return MakeWrenchLinCost(Dx::kVel, weight);
  case Parameters::WrenchAngPosCost:
    return MakeWrenchAngCost(Dx::kPos, weight);
  case Parameters::WrenchAngVelCost:
    return MakeWrenchAngCost(Dx::kVel, weight);
  case Parameters::WrenchLinVelDiffCost:
    return MakeWrenchLinVelDiffCost(weight);
  case Parameters::WrenchAngVelDiffCost:
    return MakeWrenchAngVelDiffCost(weight);
  default:
    throw std::runtime_error("cost not defined!");
  }
}

NlpFormulation::CostPtrVec
NlpFormulation::MakeBaseLinVelDiffCost(const Eigen::VectorXd &weight) const {
  CostPtrVec cost;
  // X, Y, Z
  for (int i = 0; i < 3; ++i) {
    cost.push_back(std::make_shared<NodeDifferenceCost>(
        id::base_lin_nodes, Dx::kVel, i, weight(i)));
  }
  return cost;
}

NlpFormulation::CostPtrVec
NlpFormulation::MakeBaseAngVelDiffCost(const Eigen::VectorXd &weight) const {
  CostPtrVec cost;
  // X, Y, Z
  for (int i = 0; i < 3; ++i) {
    cost.push_back(std::make_shared<NodeDifferenceCost>(
        id::base_ang_nodes, Dx::kVel, i, weight(i)));
  }
  return cost;
}

NlpFormulation::CostPtrVec
NlpFormulation::MakeWrenchLinVelDiffCost(const Eigen::VectorXd &weight) const {
  CostPtrVec cost;
  // X, Y, Z
  for (int ee = 0; ee < params_.GetEECount(); ++ee) {
    for (int i = 0; i < 3; ++i) {
      cost.push_back(std::make_shared<NodeDifferenceCost>(
          id::EEWrenchLinNodes(ee), Dx::kVel, i, weight(i)));
    }
  }
  return cost;
}

NlpFormulation::CostPtrVec
NlpFormulation::MakeWrenchAngVelDiffCost(const Eigen::VectorXd &weight) const {
  CostPtrVec cost;
  // X, Y, Z
  for (int ee = 0; ee < params_.GetEECount(); ++ee) {
    for (int i = 0; i < 3; ++i) {
      cost.push_back(std::make_shared<NodeDifferenceCost>(
          id::EEWrenchAngNodes(ee), Dx::kVel, i, weight(i)));
    }
  }
  return cost;
}

NlpFormulation::CostPtrVec
NlpFormulation::MakeWrenchLinCost(Dx dx, const Eigen::VectorXd &weight) const {
  CostPtrVec cost;
  double mg = model_.dynamic_model_->m() * model_.dynamic_model_->g();
  // For all endeffector
  for (int ee = 0; ee < params_.GetEECount(); ++ee) {
    if (dx == Dx::kPos) {
      for (int i = 0; i < 3; ++i) {
        cost.push_back(std::make_shared<NodeCost>(
            id::EEWrenchLinNodes(ee), dx, i, weight(i) / std::pow(mg, 2), 0.));
      }
    } else if (dx == Dx::kVel) {
      for (int i = 0; i < 3; ++i) {
        cost.push_back(std::make_shared<NodeCost>(id::EEWrenchLinNodes(ee), dx,
                                                  i, weight(i), 0.));
      }
    } else {
      throw std::runtime_error("[MakeWrenchLinCost] Wrong dx type");
    }
  }
  return cost;
}

NlpFormulation::CostPtrVec
NlpFormulation::MakeWrenchAngCost(Dx dx, const Eigen::VectorXd &weight) const {
  CostPtrVec cost;
  double mg = model_.dynamic_model_->m() * model_.dynamic_model_->g();
  // For all endeffector
  for (int ee = 0; ee < params_.GetEECount(); ++ee) {
    if (dx == Dx::kPos) {
      for (int i = 0; i < 3; ++i) {
        cost.push_back(std::make_shared<NodeCost>(
            id::EEWrenchAngNodes(ee), dx, i, weight(i) / std::pow(mg, 2), 0.));
      }
    } else if (dx == Dx::kVel) {
      for (int i = 0; i < 3; ++i) {
        cost.push_back(std::make_shared<NodeCost>(id::EEWrenchAngNodes(ee), dx,
                                                  i, weight(i), 0.));
      }
    } else {
      throw std::runtime_error("[MakeWrenchAngCost] Wrong dx type");
    }
  }
  return cost;
}

NlpFormulation::CostPtrVec NlpFormulation::MakeFinalEEMotionLinPosCost(
    const Eigen::VectorXd &weight) const {
  CostPtrVec cost;
  // For all endeffector
  for (int ee = 0; ee < params_.GetEECount(); ++ee) {
    double x = final_base_.lin.p().x() +
               model_.kinematic_model_->GetNominalStanceInBase().at(ee)(0);
    double y = final_base_.lin.p().y() +
               model_.kinematic_model_->GetNominalStanceInBase().at(ee)(1);
    double z = terrain_->GetHeight(x, y);
    Eigen::Vector3d final_ee_motion_lin_pos(x, y, z);
    for (int i = 0; i < 3; ++i) {
      cost.push_back(std::make_shared<FinalNodeCost>(
          id::EEMotionLinNodes(ee), kPos, i, weight(i),
          final_ee_motion_lin_pos(i)));
    }
  }

  return cost;
}

NlpFormulation::CostPtrVec NlpFormulation::MakeFinalEEMotionAngPosCost(
    const Eigen::VectorXd &weight) const {
  CostPtrVec cost;
  // For all endeffector
  for (int ee = 0; ee < params_.GetEECount(); ++ee) {
    for (int i = 0; i < 3; ++i) {
      cost.push_back(std::make_shared<FinalNodeCost>(id::EEMotionAngNodes(ee),
                                                     kPos, i, weight(i),
                                                     final_base_.ang.p()(i)));
    }
  }

  return cost;
}

NlpFormulation::CostPtrVec
NlpFormulation::MakeFinalBaseLinCost(Dx dx,
                                     const Eigen::VectorXd &weight) const {
  CostPtrVec cost;
  if (dx == Dx::kPos) {
    double x = final_base_.lin.p().x();
    double y = final_base_.lin.p().y();
    double z = terrain_->GetHeight(x, y) + final_base_.lin.p().z();
    Eigen::Vector3d final_base_lin_pos(x, y, z);
    for (int i = 0; i < 3; ++i) {
      cost.push_back(std::make_shared<FinalNodeCost>(
          id::base_lin_nodes, dx, i, weight(i), final_base_lin_pos(i)));
    }
  } else if (dx == Dx::kVel) {
    for (int i = 0; i < 3; ++i) {
      cost.push_back(std::make_shared<FinalNodeCost>(
          id::base_lin_nodes, dx, i, weight(i), final_base_.lin.v()(i)));
    }
  } else {
    throw std::runtime_error("[MakeFinalBaseLinCost] Wrong dx type");
  }

  return cost;
}

NlpFormulation::CostPtrVec NlpFormulation::MakeIntermediateBaseLinCost(
    Dx dx, const Eigen::VectorXd &weight) const {
  CostPtrVec cost;
  if (dx == Dx::kPos) {
    for (int i = 0; i < 3; ++i) {
      cost.push_back(std::make_shared<IntermediateNodeCost>(
          id::base_lin_nodes, dx, i, weight(i),
          0.5 * (initial_base_.lin.p()(i) + final_base_.lin.p()(i))));
    }
  } else if (dx == Dx::kVel) {
    for (int i = 0; i < 3; ++i) {
      cost.push_back(std::make_shared<IntermediateNodeCost>(
          id::base_lin_nodes, dx, i, weight(i), 0.));
    }
  } else {
    throw std::runtime_error("[MakeIntermediateBaseLinCost] Wrong dx type");
  }

  return cost;
}

NlpFormulation::CostPtrVec
NlpFormulation::MakeFinalBaseAngCost(Dx dx,
                                     const Eigen::VectorXd &weight) const {
  CostPtrVec cost;
  if (dx == Dx::kPos) {
    for (int i = 0; i < 3; ++i) {
      cost.push_back(std::make_shared<FinalNodeCost>(
          id::base_ang_nodes, dx, i, weight(i), final_base_.ang.p()(i)));
    }
  } else if (dx == Dx::kVel) {
    for (int i = 0; i < 3; ++i) {
      cost.push_back(std::make_shared<FinalNodeCost>(
          id::base_ang_nodes, dx, i, weight(i), final_base_.ang.v()(i)));
    }
  } else {
    throw std::runtime_error("[MakeFinalBaseAngCost] Wrong dx type");
  }

  return cost;
}

NlpFormulation::CostPtrVec NlpFormulation::MakeIntermediateBaseAngCost(
    Dx dx, const Eigen::VectorXd &weight) const {
  CostPtrVec cost;
  if (dx == Dx::kPos) {
    for (int i = 0; i < 3; ++i) {
      cost.push_back(std::make_shared<IntermediateNodeCost>(
          id::base_ang_nodes, dx, i, weight(i),
          0.5 * (initial_base_.ang.p()(i) + final_base_.ang.p()(i))));
    }
  } else if (dx == Dx::kVel) {
    for (int i = 0; i < 3; ++i) {
      cost.push_back(std::make_shared<IntermediateNodeCost>(
          id::base_ang_nodes, dx, i, weight(i), 0.));
    }
  } else {
    throw std::runtime_error("[MakeIntermediateBaseAngCost] Wrong dx type");
  }

  return cost;
}

void NlpFormulation::from_locomotion_task(const LocomotionTask &task) {
  terrain_ = task.terrain;

  initial_base_.lin.at(kPos) = task.initial_base_lin.segment(0, 3);
  initial_base_.lin.at(kVel) = task.initial_base_lin.segment(3, 3);
  initial_base_.ang.at(kPos) = task.initial_base_ang.segment(0, 3);
  initial_base_.ang.at(kVel) = task.initial_base_ang.segment(3, 3);

  initial_ee_motion_lin_ = task.initial_ee_motion_lin;
  initial_ee_motion_ang_ = task.initial_ee_motion_ang;

  final_base_.lin.at(kPos) = task.final_base_lin.segment(0, 3);
  final_base_.lin.at(kVel) = task.final_base_lin.segment(3, 3);
  final_base_.ang.at(kPos) = task.final_base_ang.segment(0, 3);
  final_base_.ang.at(kVel) = task.final_base_ang.segment(3, 3);
}

void NlpFormulation::initialize_from_dcm_planner(const std::string &traj_type) {
  // ===========================================================================
  // Initialize DCM Planner
  // ===========================================================================
  double nominal_com_height =
      -model_.kinematic_model_->GetNominalStanceInBase().at(0)[2];
  dcm_planner_.t_transfer = 0.;
  if (params_.ee_phase_durations_.at(0)[0] <=
      params_.ee_phase_durations_.at(1)[0]) {
    dcm_planner_.t_ds = params_.ee_phase_durations_.at(0)[0] / 1.5;
    dcm_planner_.t_ss = params_.ee_phase_durations_.at(0)[1];
  } else {
    dcm_planner_.t_ds = params_.ee_phase_durations_.at(1)[0];
    dcm_planner_.t_ss = params_.ee_phase_durations_.at(1)[1];
  }
  dcm_planner_.percentage_settle = 0.9;
  dcm_planner_.alpha_ds = 0.5;
  dcm_planner_.setRobotMass(model_.dynamic_model_->m());
  dcm_planner_.setCoMHeight(nominal_com_height);
  dcm_planner_.setInitialTime(0.);
  dcm_planner_.setInitialOri(euler_xyz_to_quat(
      Eigen::Vector3d(final_base_.ang.at(kPos)[0], final_base_.ang.at(kPos)[1],
                      final_base_.ang.at(kPos)[2])));

  // ===========================================================================
  // Fill footstep_list based on the trjectory type
  // ===========================================================================
  std::vector<Footstep> footstep_list;

  int n_lf = (params_.ee_phase_durations_.at(0).size() - 1) / 2;
  int n_rf = (params_.ee_phase_durations_.at(1).size() - 1) / 2;
  int robot_side;
  if (params_.ee_phase_durations_.at(0)[0] <=
      params_.ee_phase_durations_.at(1)[0]) {
    robot_side = LEFT_ROBOT_SIDE; // Lift left foot first
  } else {
    robot_side = RIGHT_ROBOT_SIDE; // Lift right foot first
  }
  Footstep left_foot_stance, right_foot_stance, mid_foot_stance;
  Eigen::Vector3d lfoot_pos = initial_ee_motion_lin_.at(0);
  Eigen::Quaternion<double> lfoot_ori(
      euler_xyz_to_rot(initial_ee_motion_ang_.at(0)));
  left_foot_stance.setPosOriSide(lfoot_pos, lfoot_ori, LEFT_ROBOT_SIDE);
  Eigen::Vector3d rfoot_pos = initial_ee_motion_lin_.at(1);
  Eigen::Quaternion<double> rfoot_ori(
      euler_xyz_to_rot(initial_ee_motion_ang_.at(1)));
  right_foot_stance.setPosOriSide(rfoot_pos, rfoot_ori, RIGHT_ROBOT_SIDE);
  mid_foot_stance.computeMidfeet(left_foot_stance, right_foot_stance,
                                 mid_foot_stance);

  if (traj_type == "dubins") {
    DubinsPath dp;
    double q0[] = {initial_base_.lin.at(kPos)(0), initial_base_.lin.at(kPos)(1),
                   initial_base_.ang.at(kPos)(2)};
    double q1[] = {final_base_.lin.at(kPos)(0), final_base_.lin.at(kPos)(1),
                   final_base_.ang.at(kPos)(2)};
    double turning_radius = 0.25;
    // double turning_radius = 0.5;
    int res = dubins_shortest_path(&dp, q0, q1, turning_radius);
    assert(res == 0);
    double total_dp_len = dubins_path_length(&dp);

    for (int i = 0; i < n_lf + n_rf; ++i) {
      Eigen::Vector3d base_pos, ee_pos;
      Eigen::Quaternion<double> ee_quat;
      Eigen::Matrix3d w_R_b;
      if (i < n_lf + n_rf - 2) {
        // Intermediate Steps
        double t = (i + 1) * total_dp_len /
                   (n_lf + n_rf - 1); // Since the last step is recovering step
        double qt[3];                 // (x, y, yaw)
        dubins_path_sample(&dp, t, qt);
        base_pos << qt[0], qt[1],
            terrain_->GetHeight(qt[0], qt[1]) + nominal_com_height;
        w_R_b = euler_xyz_to_rot(Eigen::Vector3d(0., 0., qt[2]));

      } else {
        // Final Recovery Steps
        double com_x = final_base_.lin.p().x();
        double com_y = final_base_.lin.p().y();
        double com_z = final_base_.lin.p().z();
        base_pos << com_x, com_y, terrain_->GetHeight(com_x, com_y) + com_z;
        w_R_b = euler_xyz_to_rot(Eigen::Vector3d(final_base_.ang.p().x(),
                                                 final_base_.ang.p().y(),
                                                 final_base_.ang.p().z()));
      }
      ee_quat = w_R_b;
      if (robot_side == LEFT_ROBOT_SIDE) {
        ee_pos =
            base_pos +
            w_R_b * model_.kinematic_model_->GetNominalStanceInBase().at(0);
        ee_pos[2] = terrain_->GetHeight(ee_pos[0], ee_pos[1]);
        footstep_list.push_back(Footstep(ee_pos, ee_quat, robot_side));
        robot_side = RIGHT_ROBOT_SIDE;
      } else {
        ee_pos =
            base_pos +
            w_R_b * model_.kinematic_model_->GetNominalStanceInBase().at(1);
        ee_pos[2] = terrain_->GetHeight(ee_pos[0], ee_pos[1]);
        footstep_list.push_back(Footstep(ee_pos, ee_quat, robot_side));
        robot_side = LEFT_ROBOT_SIDE;
      }
    }
  } else if (traj_type == "turning") {

    double turn_radians_per_step =
        final_base_.ang.p()[2] / ((n_lf + n_rf) / 2.);
    Eigen::Quaterniond foot_rotate(
        Eigen::AngleAxisd(turn_radians_per_step, Eigen::Vector3d::UnitZ()));

    Footstep left_footstep, right_footstep;
    Footstep mid_footstep = mid_foot_stance;
    Footstep mid_footstep_rotated = mid_footstep;
    assert(n_lf == n_rf);
    for (int i = 0; i < (n_lf + n_rf) / 2; ++i) {
      mid_footstep_rotated.setPosOri(mid_footstep.position,
                                     foot_rotate * mid_footstep.orientation);

      left_footstep.setPosOriSide(
          mid_footstep_rotated.position +
              mid_footstep_rotated.R_ori *
                  Eigen::Vector3d(0, initial_ee_motion_lin_.at(0)[1], 0),
          mid_footstep_rotated.orientation, LEFT_ROBOT_SIDE);
      right_footstep.setPosOriSide(
          mid_footstep_rotated.position +
              mid_footstep_rotated.R_ori *
                  Eigen::Vector3d(0, initial_ee_motion_lin_.at(1)[1], 0),
          mid_footstep_rotated.orientation, RIGHT_ROBOT_SIDE);

      if (turn_radians_per_step > 0) {
        footstep_list.push_back(left_footstep);
        footstep_list.push_back(right_footstep);
      } else {
        footstep_list.push_back(right_footstep);
        footstep_list.push_back(left_footstep);
      }
      mid_footstep = mid_footstep_rotated;
    }
  } else if (traj_type == "side") {
    double strafe_distance = final_base_.lin.p()[1] / ((n_lf + n_rf) / 2.);
    Footstep left_footstep, right_footstep;
    Footstep mid_footstep = mid_foot_stance;
    Footstep mid_footstep_translated = mid_footstep;
    assert(n_lf == n_rf);
    for (int i = 0; i < (n_lf + n_rf) / 2; ++i) {
      mid_footstep_translated.setPosOri(
          mid_footstep.position +
              mid_footstep.R_ori * Eigen::Vector3d(0.0, strafe_distance, 0.0),
          mid_footstep.orientation);

      left_footstep.setPosOriSide(
          mid_footstep_translated.position +
              mid_footstep_translated.R_ori *
                  Eigen::Vector3d(0, initial_ee_motion_lin_.at(0)[1], 0),
          mid_footstep_translated.orientation, LEFT_ROBOT_SIDE);
      right_footstep.setPosOriSide(
          mid_footstep_translated.position +
              mid_footstep_translated.R_ori *
                  Eigen::Vector3d(0, initial_ee_motion_lin_.at(1)[1], 0),
          mid_footstep_translated.orientation, RIGHT_ROBOT_SIDE);

      if (strafe_distance > 0) {
        // Left strafe
        footstep_list.push_back(left_footstep);
        footstep_list.push_back(right_footstep);
      } else {
        // Right strafe
        footstep_list.push_back(right_footstep);
        footstep_list.push_back(left_footstep);
      }
      mid_footstep = mid_footstep_translated;
    }
  } else if (traj_type == "round") {

    // n_lf = 9;
    // n_rf = 9;
    double turn_radians_per_step = 10. * M_PI / 180.;
    Eigen::Quaterniond foot_rotate(
        Eigen::AngleAxisd(10. * M_PI / 180., Eigen::Vector3d::UnitZ()));
    double strafe_distance = -0.07;

    // double turn_radians_per_step =
    // final_base_.ang.p()[2] / ((n_lf + n_rf) / 2.);
    // Eigen::Quaterniond foot_rotate(
    // Eigen::AngleAxisd(turn_radians_per_step, Eigen::Vector3d::UnitZ()));
    // double strafe_distance = final_base_.lin.p()[1] / ((n_lf + n_rf) / 2.);

    // std::cout << n_lf << std::endl;
    // std::cout << n_rf << std::endl;
    // std::cout << turn_radians_per_step << std::endl;
    // std::cout << strafe_distance << std::endl;
    // exit(0);

    Footstep left_footstep, right_footstep;
    Footstep mid_footstep = mid_foot_stance;
    Footstep mid_footstep_translated = mid_footstep;
    assert(n_lf == n_rf);
    for (int i = 0; i < (n_lf + n_rf) / 2; ++i) {
      mid_footstep_translated.setPosOri(
          mid_footstep.position +
              mid_footstep.R_ori * Eigen::Vector3d(0.0, strafe_distance, 0.0),
          foot_rotate * mid_footstep.orientation);

      left_footstep.setPosOriSide(
          mid_footstep_translated.position +
              mid_footstep_translated.R_ori *
                  Eigen::Vector3d(0, initial_ee_motion_lin_.at(0)[1], 0),
          mid_footstep_translated.orientation, LEFT_ROBOT_SIDE);
      right_footstep.setPosOriSide(
          mid_footstep_translated.position +
              mid_footstep_translated.R_ori *
                  Eigen::Vector3d(0, initial_ee_motion_lin_.at(1)[1], 0),
          mid_footstep_translated.orientation, RIGHT_ROBOT_SIDE);

      if (turn_radians_per_step > 0) {
        // Left strafe
        footstep_list.push_back(left_footstep);
        footstep_list.push_back(right_footstep);
      } else {
        // Right strafe
        footstep_list.push_back(right_footstep);
        footstep_list.push_back(left_footstep);
      }
      mid_footstep = mid_footstep_translated;
    }
  } else {
    assert(false); // Wrong type
  }
  Footstep left_foot_start = Footstep(
      initial_ee_motion_lin_.at(0),
      euler_xyz_to_quat(initial_ee_motion_ang_.at(0)), LEFT_ROBOT_SIDE);
  Footstep right_foot_start = Footstep(
      initial_ee_motion_lin_.at(1),
      euler_xyz_to_quat(initial_ee_motion_ang_.at(1)), RIGHT_ROBOT_SIDE);

  // Solve
  double dcm_omega = sqrt(model_.dynamic_model_->g() / nominal_com_height);
  Eigen::Vector3d dcm_pos_start;
  for (int i = 0; i < 3; ++i) {
    dcm_pos_start[i] = (initial_base_.lin.at(kPos) +
                        initial_base_.lin.at(kVel) / dcm_omega)[i];
  }
  Eigen::Vector3d dcm_vel_start(0., 0., 0.);
  dcm_planner_.initialize_footsteps_rvrp(footstep_list, left_foot_start,
                                         right_foot_start, dcm_pos_start,
                                         dcm_vel_start);

  // for (int i = 0; i < footstep_list.size(); ++i)
  // footstep_list[i].printInfo();

  // ===========================================================================
  // Get solutions and fill one-hot vectors
  // ===========================================================================
  // Base Vars
  int n_base_nodes = params_.GetBasePolyDurations().size() + 1;
  int n_base_vars = n_base_nodes * 6;
  one_hot_base_lin_ = Eigen::VectorXd::Zero(n_base_vars);
  one_hot_base_ang_ = Eigen::VectorXd::Zero(n_base_vars);
  double t(0.);
  for (int i = 0; i < n_base_nodes; ++i) {
    Eigen::Vector3d com_pos, com_vel, base_ang_vel, base_ang_acc;
    Eigen::Quaternion<double> base_quat;
    dcm_planner_.get_ref_com(t, com_pos);
    dcm_planner_.get_ref_com_vel(t, com_vel);
    dcm_planner_.get_ref_ori_ang_vel_acc(t, base_quat, base_ang_vel,
                                         base_ang_acc);
    one_hot_base_lin_.segment(i * 6, 3) = com_pos;
    one_hot_base_lin_.segment(i * 6 + 3, 3) = com_vel;
    one_hot_base_ang_.segment(i * 6, 3) = quat_to_euler_xyz(base_quat);
    one_hot_base_ang_.segment(i * 6 + 3, 3) = base_ang_vel;
    if (i != (n_base_nodes - 1))
      t += params_.GetBasePolyDurations()[i];
  }

  // Motion Vars
  std::vector<int> n_ee_motion_lin_vars_(2);
  std::vector<int> n_ee_motion_ang_vars_(2);
  // First (Contact)
  std::vector<Eigen::Vector3d> prev_pos(2);
  std::vector<Eigen::Vector3d> prev_euler_angles(2);
  for (auto ee : {L, R}) {
    n_ee_motion_lin_vars_.at(ee) =
        3 * (params_.ee_phase_durations_.at(ee).size() + 1) / 2 +
        5 * (params_.ee_phase_durations_.at(ee).size() - 1) / 2;
    n_ee_motion_ang_vars_.at(ee) =
        3 * (params_.ee_phase_durations_.at(ee).size() + 1) / 2 +
        6 * (params_.ee_phase_durations_.at(ee).size() - 1) / 2;
    one_hot_ee_motion_lin_.at(ee) =
        Eigen::VectorXd::Zero(n_ee_motion_lin_vars_.at(ee));
    one_hot_ee_motion_ang_.at(ee) =
        Eigen::VectorXd::Zero(n_ee_motion_ang_vars_.at(ee));
    Eigen::Vector3d pos, euler_angle;
    if (ee == L) {
      pos = left_foot_start.position;
      euler_angle = quat_to_euler_xyz(left_foot_start.orientation);
    } else {
      pos = right_foot_start.position;
      euler_angle = quat_to_euler_xyz(right_foot_start.orientation);
    }
    one_hot_ee_motion_lin_.at(ee).segment(0, 3) = pos;
    one_hot_ee_motion_ang_.at(ee).segment(0, 3) = euler_angle;
    prev_pos.at(ee) = pos;
    prev_euler_angles.at(ee) = euler_angle;
  }
  // Rest (Swing Contact)
  double swing_height = 0.05;
  double t_avg_swing = params_.ee_phase_durations_.at(0)[1];
  int ee = 0;
  std::vector<int> id(2, 0);
  for (int i = 0; i < footstep_list.size(); ++i) {
    int robot_side = footstep_list[i].robot_side;
    if (robot_side == LEFT_ROBOT_SIDE) {
      ee = 0;
    } else {
      ee = 1;
    }
    Eigen::Vector3d pos = footstep_list[i].position;
    Eigen::Vector3d euler_angles =
        quat_to_euler_xyz(footstep_list[i].orientation);
    Eigen::VectorXd lin_var = Eigen::VectorXd::Zero(8);
    Eigen::VectorXd ang_var = Eigen::VectorXd::Zero(9);
    Eigen::Vector3d mid_pos = (prev_pos.at(ee) + pos) / 2.;
    Eigen::Vector3d mid_vel = (pos - prev_pos.at(ee)) / t_avg_swing;
    Eigen::Vector3d mid_ang = (prev_euler_angles.at(ee) + euler_angles) / 2.;
    Eigen::Vector3d mid_ang_vel =
        (euler_angles - prev_euler_angles.at(ee)) / t_avg_swing;
    // (X, DX, Y, DY, Z, X, Y, Z)
    lin_var << mid_pos[0], mid_vel[0], mid_pos[1], mid_vel[1], swing_height,
        pos[0], pos[1], pos[2];
    // (X, DX, Y, DY, Z, DZ, X, Y, Z)
    ang_var << mid_ang[0], mid_ang_vel[0], mid_ang[1], mid_ang_vel[1],
        mid_ang[2], mid_ang_vel[2], euler_angles[0], euler_angles[1],
        euler_angles[2];
    one_hot_ee_motion_lin_.at(ee).segment(3 + id.at(ee) * 8, 8) = lin_var;
    one_hot_ee_motion_ang_.at(ee).segment(3 + id.at(ee)++ * 9, 9) = ang_var;

    prev_pos.at(ee) = pos;
    prev_euler_angles.at(ee) = euler_angles;
  }

  // Reaction Force Vars

  std::vector<int> n_ee_wrench_vars_(2);
  for (auto ee : {L, R}) {
    n_ee_wrench_vars_.at(ee) =
        6 * (2 * params_.force_polynomials_per_stance_phase_ +
             (((params_.ee_phase_durations_.at(ee).size() + 1) / 2) - 2) *
                 (params_.force_polynomials_per_stance_phase_ - 1));
    one_hot_ee_wrench_lin_.at(ee) =
        Eigen::VectorXd::Zero(n_ee_wrench_vars_.at(ee));
    one_hot_ee_wrench_ang_.at(ee) =
        Eigen::VectorXd::Zero(n_ee_wrench_vars_.at(ee));

    int starting_idx(0);
    for (int j = 0; j < params_.ee_phase_durations_.at(ee).size(); ++j) {
      if (j % 2 == 0) {
        double t_so_far(0.);
        for (int i = 0; i < j; ++i) {
          t_so_far += params_.ee_phase_durations_.at(ee)[i];
        }
        Eigen::VectorXd lf_wrench, rf_wrench;
        std::vector<Eigen::Vector3d> frcs(4);
        std::vector<Eigen::Vector3d> frcs_d(4);
        std::vector<Eigen::Vector3d> trqs(4);
        std::vector<Eigen::Vector3d> trqs_d(4);
        for (int quarter_id = 0; quarter_id < 4; ++quarter_id) {
          double t = t_so_far +
                     quarter_id / 3. * params_.ee_phase_durations_.at(ee)[j];
          dcm_planner_.get_ref_reaction_force(t, lf_wrench, rf_wrench);
          if (ee == 0) {
            trqs[quarter_id] = lf_wrench.head(3);
            frcs[quarter_id] = lf_wrench.tail(3);
          } else {
            trqs[quarter_id] = rf_wrench.head(3);
            frcs[quarter_id] = rf_wrench.tail(3);
          }
        }
        frcs_d[0].setZero();
        frcs_d[1] =
            (frcs[0] + frcs[2]) / 3. * params_.ee_phase_durations_.at(ee)[j];
        frcs_d[2] = (frcs[1]) / 3. * params_.ee_phase_durations_.at(ee)[j];
        frcs_d[3].setZero();
        trqs_d[0].setZero();
        trqs_d[1] =
            (trqs[0] + trqs[2]) / 3. * params_.ee_phase_durations_.at(ee)[j];
        trqs_d[2] = (trqs[1]) / 3. * params_.ee_phase_durations_.at(ee)[j];
        trqs_d[3].setZero();

        if (params_.ee_phase_durations_.at(ee).size() == 1) {
          one_hot_ee_wrench_lin_.at(ee) << frcs.at(0)[0], frcs_d.at(0)[0],
              frcs.at(0)[1], frcs_d.at(0)[1], frcs.at(0)[2], frcs_d.at(0)[2],
              frcs.at(1)[0], frcs_d.at(1)[0], frcs.at(1)[1], frcs_d.at(1)[1],
              frcs.at(1)[2], frcs_d.at(1)[2], frcs.at(2)[0], frcs_d.at(2)[0],
              frcs.at(2)[1], frcs_d.at(2)[1], frcs.at(2)[2], frcs_d.at(2)[2],
              frcs.at(3)[0], frcs_d.at(3)[0], frcs.at(3)[1], frcs_d.at(3)[1],
              frcs.at(3)[2], frcs_d.at(3)[2];

          one_hot_ee_wrench_ang_.at(ee) << trqs.at(0)[0], trqs_d.at(0)[0],
              trqs.at(0)[1], trqs_d.at(0)[1], trqs.at(0)[2], trqs_d.at(0)[2],
              trqs.at(1)[0], trqs_d.at(1)[0], trqs.at(1)[1], trqs_d.at(1)[1],
              trqs.at(1)[2], trqs_d.at(1)[2], trqs.at(2)[0], trqs_d.at(2)[0],
              trqs.at(2)[1], trqs_d.at(2)[1], trqs.at(2)[2], trqs_d.at(2)[2],
              trqs.at(3)[0], trqs_d.at(3)[0], trqs.at(3)[1], trqs_d.at(3)[1],
              trqs.at(3)[2], trqs_d.at(3)[2];
        } else if (j == 0) {
          Eigen::VectorXd tmp_vec = Eigen::VectorXd::Zero(18);
          tmp_vec << frcs.at(0)[0], frcs_d.at(0)[0], frcs.at(0)[1],
              frcs_d.at(0)[1], frcs.at(0)[2], frcs_d.at(0)[2], frcs.at(1)[0],
              frcs_d.at(1)[0], frcs.at(1)[1], frcs_d.at(1)[1], frcs.at(1)[2],
              frcs_d.at(1)[2], frcs.at(2)[0], frcs_d.at(2)[0], frcs.at(2)[1],
              frcs_d.at(2)[1], frcs.at(2)[2], frcs_d.at(2)[2];
          one_hot_ee_wrench_lin_.at(ee).segment(starting_idx, 18) = tmp_vec;

          tmp_vec << trqs.at(0)[0], trqs_d.at(0)[0], trqs.at(0)[1],
              trqs_d.at(0)[1], trqs.at(0)[2], trqs_d.at(0)[2], trqs.at(1)[0],
              trqs_d.at(1)[0], trqs.at(1)[1], trqs_d.at(1)[1], trqs.at(1)[2],
              trqs_d.at(1)[2], trqs.at(2)[0], trqs_d.at(2)[0], trqs.at(2)[1],
              trqs_d.at(2)[1], trqs.at(2)[2], trqs_d.at(2)[2];
          one_hot_ee_wrench_ang_.at(ee).segment(starting_idx, 18) = tmp_vec;

          starting_idx += 18;
        } else if (j == params_.ee_phase_durations_.at(ee).size() - 1) {

          Eigen::VectorXd tmp_vec = Eigen::VectorXd::Zero(18);
          tmp_vec << frcs.at(1)[0], frcs_d.at(1)[0], frcs.at(1)[1],
              frcs_d.at(1)[1], frcs.at(1)[2], frcs_d.at(1)[2], frcs.at(2)[0],
              frcs_d.at(2)[0], frcs.at(2)[1], frcs_d.at(2)[1], frcs.at(2)[2],
              frcs_d.at(2)[2], frcs.at(3)[0], frcs_d.at(3)[0], frcs.at(3)[1],
              frcs_d.at(3)[1], frcs.at(3)[2], frcs_d.at(3)[2];
          one_hot_ee_wrench_lin_.at(ee).segment(starting_idx, 18) = tmp_vec;

          tmp_vec << trqs.at(1)[0], trqs_d.at(1)[0], trqs.at(1)[1],
              trqs_d.at(1)[1], trqs.at(1)[2], trqs_d.at(1)[2], trqs.at(2)[0],
              trqs_d.at(2)[0], trqs.at(2)[1], trqs_d.at(2)[1], trqs.at(2)[2],
              trqs_d.at(2)[2], trqs.at(3)[0], trqs_d.at(3)[0], trqs.at(3)[1],
              trqs_d.at(3)[1], trqs.at(3)[2], trqs_d.at(3)[2];
          one_hot_ee_wrench_ang_.at(ee).segment(starting_idx, 18) = tmp_vec;

          starting_idx += 18;
        } else {

          Eigen::VectorXd tmp_vec = Eigen::VectorXd::Zero(12);
          tmp_vec << frcs.at(1)[0], frcs_d.at(1)[0], frcs.at(1)[1],
              frcs_d.at(1)[1], frcs.at(1)[2], frcs_d.at(1)[2], frcs.at(2)[0],
              frcs_d.at(2)[0], frcs.at(2)[1], frcs_d.at(2)[1], frcs.at(2)[2],
              frcs_d.at(2)[2];
          one_hot_ee_wrench_lin_.at(ee).segment(starting_idx, 12) = tmp_vec;

          tmp_vec << trqs.at(1)[0], trqs_d.at(1)[0], trqs.at(1)[1],
              trqs_d.at(1)[1], trqs.at(1)[2], trqs_d.at(1)[2], trqs.at(2)[0],
              trqs_d.at(2)[0], trqs.at(2)[1], trqs_d.at(2)[1], trqs.at(2)[2],
              trqs_d.at(2)[2];
          one_hot_ee_wrench_ang_.at(ee).segment(starting_idx, 12) = tmp_vec;

          starting_idx += 12;
        }
      } else {
      }
    }
  }

  b_initialize_ = true;
}

} /* namespace towr_plus */
