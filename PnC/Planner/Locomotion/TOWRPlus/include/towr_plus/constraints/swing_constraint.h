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

#ifndef TOWR_CONSTRAINTS_SWING_CONSTRAINT_H_
#define TOWR_CONSTRAINTS_SWING_CONSTRAINT_H_

#include <ifopt/constraint_set.h>
#include <towr_plus/variables/nodes_variables_phase_based.h>

namespace towr_plus {

/**
 * @brief Constrains the foot position during the swing-phase.
 *
 * This avoids very quick swinging of the feet, where the polynomial then
 * leaves the e.g. range-of-motion in between nodes. This constraint can also
 * be used to force a leg lift. However, it is cleanest if the optimization
 * can be performed without this heuristic constraint.
 *
 * @ingroup Constraints
 */
class SwingConstraint : public ifopt::ConstraintSet {
public:
  using Vector2d = Eigen::Vector2d;

  /**
   * @brief Links the swing constraint with current foot variables.
   * @param ee_motion_id  The name of the foot variables in the optimization.
   */
  SwingConstraint(int ee, std::string ee_motion_linear_id,
                  std::string ee_motion_angular_id, double t_swing_avg = 0.5);
  virtual ~SwingConstraint() = default;

  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock(std::string var_set, Jacobian &) const override;

  void InitVariableDependedQuantities(const VariablesPtr &x) override;

private:
  NodesVariablesPhaseBased::Ptr ee_motion_linear_;
  NodesVariablesPhaseBased::Ptr ee_motion_angular_;
  double t_swing_avg_;
  std::string ee_motion_linear_id_;
  std::string ee_motion_angular_id_;

  std::vector<int> pure_swing_node_ids_;

  int lin_constraint_count_;
  int ang_constraint_count_;
};

} /* namespace towr_plus */

#endif /* TOWR_CONSTRAINTS_SWING_CONSTRAINT_H_ */
