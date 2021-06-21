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

#pragma once

#include <towr_plus/models/kinematic_model.h>
#include <towr_plus/variables/euler_converter.h>
#include <towr_plus/variables/spline.h>
#include <towr_plus/variables/spline_holder.h>

#include "time_discretization_constraint.h"

namespace towr_plus {

/** @brief Constrains an endeffector to lie in a box around the nominal stance.
 *
 * These constraints are necessary to avoid configurations
 * that are outside the kinematic reach of the robot. The constraint
 * is defined by Cartesian estimates of the reachability of each endeffector.
 *
 * This constraint calculates the position of of the contact expressed in the
 * current CoM frame and constrains it to lie in a box around the nominal/
 * natural contact position for that leg.
 *
 * @ingroup Constraints
 */
class RangeOfMotionCircleConstraint : public TimeDiscretizationConstraint {
public:
  using EE = uint;
  using Vector3d = Eigen::Vector3d;

  /**
   * @brief Constructs a constraint instance.
   * @param robot_model   The kinematic restrictions of the robot.
   * @param T   The total duration of the optimization.
   * @param dt  the discretization intervall at which to enforce constraints.
   * @param ee            The endeffector for which to constrain the range.
   * @param spline_holder Pointer to the current variables.
   */
  RangeOfMotionCircleConstraint(const KinematicModel::Ptr &robot_model,
                                double T, double dt, const EE &ee,
                                const SplineHolder &spline_holder);
  virtual ~RangeOfMotionCircleConstraint() = default;

private:
  NodeSpline::Ptr base_linear_;
  EulerConverter base_angular_;
  NodeSpline::Ptr ee_motion_linear_;
  EulerConverter ee_motion_angular_;

  Eigen::Vector3d max_deviation_from_nominal_;
  Eigen::Vector3d min_deviation_from_nominal_;
  Eigen::Vector3d nominal_ee_pos_B_;
  double nominal_base_to_ee_len_;
  EE ee_;

  // see TimeDiscretizationConstraint for documentation
  void UpdateConstraintAtInstance(double t, int k, VectorXd &g) const override;
  void UpdateBoundsAtInstance(double t, int k, VecBound &) const override;
  void UpdateJacobianAtInstance(double t, int k, std::string,
                                Jacobian &) const override;

  int GetRow(int node, int dimension) const;

  double max_len_;
  double min_len_;
  double min_cos_;
  double max_cos_;

  Eigen::MatrixXd S1_;      // Selection matrix
  Eigen::MatrixXd S2_;      // Selection matrix
  Eigen::MatrixXd local_x_; // base local x axis

  EulerConverter::MatrixSXd S1__;      // Selection matrix
  EulerConverter::MatrixSXd S2__;      // Selection matrix
  EulerConverter::MatrixSXd local_x__; // base local x axis
};

} /* namespace towr_plus */
