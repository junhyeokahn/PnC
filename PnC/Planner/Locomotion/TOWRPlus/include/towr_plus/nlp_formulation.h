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

#ifndef TOWR_NLP_FACTORY_H_
#define TOWR_NLP_FACTORY_H_

#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <ifopt/variable_set.h>
#include <towr_plus/initialization/dcm_planner.hpp>
#include <towr_plus/locomotion_task.h>
#include <towr_plus/models/robot_model.h>
#include <towr_plus/parameters.h>
#include <towr_plus/terrain/height_map.h>
#include <towr_plus/variables/spline_holder.h>

namespace towr_plus {

/**
 * @defgroup Constraints
 * @brief Constraints of the trajectory optimization problem.
 *
 * These are the constraint sets that characterize legged locomotion.
 *
 * Folder: @ref include/towr/constraints
 */

/**
 * @defgroup Costs
 * @brief Costs of the trajectory optimization problem.
 *
 * These the the cost terms that prioritize certain solutions to the
 * legged locomotion problem.
 *
 * Folder: @ref include/towr/costs
 */

/**
 *
 * @brief A sample combination of variables, cost and constraints.
 *
 * This is _one_ example of how to combine the variables, constraints and costs
 * provided by this library. Additional variables or constraints can be added
 * to the NLP, or existing elements replaced to find a more powerful/general
 * formulation. This formulation was used to generate the motions described
 * in this paper: https://ieeexplore.ieee.org/document/8283570/
 */

// Usage : set model --> set params --> set task
class NlpFormulation {
public:
  using VariablePtrVec = std::vector<ifopt::VariableSet::Ptr>;
  using ContraintPtrVec = std::vector<ifopt::ConstraintSet::Ptr>;
  using CostPtrVec = std::vector<ifopt::CostTerm::Ptr>;
  using EEPos = std::vector<Eigen::Vector3d>;
  using EEOri = std::vector<Eigen::Vector3d>;
  using Vector3d = Eigen::Vector3d;

  NlpFormulation();
  virtual ~NlpFormulation() = default;

  /**
   * @brief The ifopt variable sets that will be optimized over.
   * @param[in/out] builds fully-constructed splines from the variables.
   */
  VariablePtrVec GetVariableSets(SplineHolder &spline_holder);

  /**
   * @brief The ifopt constraints that enforce feasible motions.
   * @param[in] uses the fully-constructed splines for initialization of
   * constraints.
   */
  ContraintPtrVec GetConstraints(const SplineHolder &spline_holder) const;

  /** @brief The ifopt costs to tune the motion. */
  ContraintPtrVec GetCosts() const;

  BaseState initial_base_;
  BaseState final_base_;
  EEPos initial_ee_motion_lin_;
  EEOri initial_ee_motion_ang_;
  RobotModel model_;
  HeightMap::Ptr terrain_;
  Parameters params_;

  double rom_buf;

  void from_locomotion_task(const LocomotionTask &task);
  // traj_type : "dubins"
  void initialize_from_dcm_planner(const std::string &traj_type);

private:
  // variables
  std::vector<NodesVariables::Ptr> MakeBaseVariables() const;
  std::vector<NodesVariablesPhaseBased::Ptr> MakeEEMotionLinVariables() const;
  std::vector<NodesVariablesPhaseBased::Ptr> MakeEEMotionAngVariables() const;
  std::vector<NodesVariablesPhaseBased::Ptr> MakeWrenchLinVariables() const;
  std::vector<NodesVariablesPhaseBased::Ptr> MakeWrenchAngVariables() const;
  std::vector<PhaseDurations::Ptr> MakeContactScheduleVariables() const;

  // constraints
  ContraintPtrVec GetConstraint(Parameters::ConstraintName name,
                                const SplineHolder &splines) const;
  ContraintPtrVec MakeDynamicConstraint(const SplineHolder &s) const;
  ContraintPtrVec MakeRangeOfMotionBoxConstraint(const SplineHolder &s) const;
  ContraintPtrVec MakeTotalTimeConstraint() const;
  ContraintPtrVec MakeTerrainConstraint() const;
  ContraintPtrVec MakeForceConstraint(const SplineHolder &s) const;
  ContraintPtrVec MakeSwingConstraint() const;
  ContraintPtrVec MakeBaseRangeOfMotionConstraint(const SplineHolder &s) const;
  ContraintPtrVec MakeBaseAccConstraint(const SplineHolder &s) const;

  // costs
  CostPtrVec GetCost(const Parameters::CostName &id,
                     const Eigen::VectorXd weight) const;
  CostPtrVec MakeFinalBaseLinCost(Dx dx, const Eigen::VectorXd &weight) const;
  CostPtrVec MakeFinalBaseAngCost(Dx dx, const Eigen::VectorXd &weight) const;
  CostPtrVec MakeFinalEEMotionLinPosCost(const Eigen::VectorXd &weight) const;
  CostPtrVec MakeFinalEEMotionAngPosCost(const Eigen::VectorXd &weight) const;

  CostPtrVec MakeIntermediateBaseLinCost(Dx dx,
                                         const Eigen::VectorXd &weight) const;
  CostPtrVec MakeIntermediateBaseAngCost(Dx dx,
                                         const Eigen::VectorXd &weight) const;

  CostPtrVec MakeBaseLinVelDiffCost(const Eigen::VectorXd &weight) const;
  CostPtrVec MakeBaseAngVelDiffCost(const Eigen::VectorXd &weight) const;

  CostPtrVec MakeWrenchLinCost(Dx dx, const Eigen::VectorXd &weight) const;
  CostPtrVec MakeWrenchAngCost(Dx dx, const Eigen::VectorXd &weight) const;

  CostPtrVec MakeWrenchLinVelDiffCost(const Eigen::VectorXd &weight) const;
  CostPtrVec MakeWrenchAngVelDiffCost(const Eigen::VectorXd &weight) const;

  // warm starting
  bool b_initialize_;

  DCMPlanner dcm_planner_;

  Eigen::VectorXd one_hot_base_lin_;
  Eigen::VectorXd one_hot_base_ang_;
  std::vector<Eigen::VectorXd> one_hot_ee_motion_lin_;
  std::vector<Eigen::VectorXd> one_hot_ee_motion_ang_;
  std::vector<Eigen::VectorXd> one_hot_ee_wrench_lin_;
  std::vector<Eigen::VectorXd> one_hot_ee_wrench_ang_;
};

} /* namespace towr_plus */

#endif /* TOWR_NLP_FACTORY_H_ */
