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

#include <math.h> // fabs

#include <towr_plus/models/endeffector_mappings.h>
#include <towr_plus/parameters.h>
#include <towr_plus/variables/cartesian_dimensions.h>

#include <algorithm>
#include <cassert>
#include <numeric> // std::accumulate

namespace towr_plus {

Parameters::Parameters() {
  // Default Values
  duration_base_polynomial_ = 0.1;
  force_polynomials_per_stance_phase_ = 3;
  ee_polynomials_per_swing_phase_ = 2; // so step can at least lift leg

  force_limit_in_normal_direction_ = 1000;
  dt_constraint_range_of_motion_ = 0.08;
  dt_constraint_dynamic_ = 0.05;
  dt_constraint_base_motion_ = duration_base_polynomial_ / 4.;
  b_optimize_timings = true;
  bound_phase_duration_ = std::make_pair(0.2, 2.0);

  w_FinalBaseLinPosCost << 1., 1., 1.;
  w_FinalBaseLinVelCost << 1., 1., 1.;
  w_FinalBaseAngPosCost << 1., 1., 1.;
  w_FinalBaseAngVelCost << 1., 1., 1.;
  w_FinalEEMotionLinPosCost << 1., 1., 1.;
  w_FinalEEMotionAngPosCost << 1., 1., 1.;
  w_IntermediateBaseLinVelCost << 0.1, 0.1, 0.1;
  w_IntermediateBaseAngVelCost << 0.1, 0.1, 0.1;
  w_BaseLinVelDiffCost << 0.01, 0.01, 0.01;
  w_BaseAngVelDiffCost << 0.01, 0.01, 0.01;
  w_WrenchLinPosCost << 0.01, 0.01, 0.01;
  w_WrenchLinVelCost << 0.01, 0.01, 0.01;
  w_WrenchAngPosCost << 0.01, 0.01, 0.01;
  w_WrenchAngVelCost << 0.01, 0.01, 0.01;
  w_WrenchLinVelDiffCost << 0.01, 0.01, 0.01;
  w_WrenchAngVelDiffCost << 0.01, 0.01, 0.01;

  initialize();
}
void Parameters::initialize() {

  // ===========================================================================
  // Constraints
  // ===========================================================================
  constraints_.clear();
  constraints_.push_back(Terrain);
  constraints_.push_back(Dynamic);
  constraints_.push_back(BaseAcc);
  constraints_.push_back(BaseRom);
  constraints_.push_back(EndeffectorRom);
  constraints_.push_back(Force);
  constraints_.push_back(Swing);
  if (b_optimize_timings)
    constraints_.push_back(TotalTime);

  // ===========================================================================
  // Costs
  // ===========================================================================

  costs_.clear();
  // costs_.push_back({FinalBaseLinPosCost, w_FinalBaseLinPosCost});
  // costs_.push_back({FinalBaseLinVelCost, w_FinalBaseLinVelCost});
  // costs_.push_back({FinalBaseAngPosCost, w_FinalBaseAngPosCost});
  // costs_.push_back({FinalBaseAngVelCost, w_FinalBaseAngVelCost});
  // costs_.push_back({FinalEEMotionLinPosCost, w_FinalEEMotionLinPosCost});
  // costs_.push_back({FinalEEMotionAngPosCost, w_FinalEEMotionAngPosCost})
  // costs_.push_back({IntermediateBaseLinVelCost,
  // w_IntermediateBaseLinVelCost});
  // costs_.push_back({IntermediateBaseAngVelCost,
  // w_IntermediateBaseAngVelCost});
  // costs_.push_back({BaseLinVelDiffCost, w_BaseLinVelDiffCost});
  // costs_.push_back({BaseAngVelDiffCost, w_BaseAngVelDiffCost});
  // costs_.push_back({WrenchLinPosCost, w_WrenchLinPosCost});
  // costs_.push_back({WrenchLinVelCost, w_WrenchLinVelCost});
  // costs_.push_back({WrenchLinVelDiffCost, w_WrenchLinVelDiffCost});
  // costs_.push_back({WrenchAngVelDiffCost, w_WrenchAngVelDiffCost});

  // costs_.push_back({WrenchAngPosCost, w_WrenchAngPosCost});
  // costs_.push_back({WrenchAngVelCost, w_WrenchAngVelCost});

  // ===========================================================================
  // Final Boundary Constarints
  // ===========================================================================
  // bounds_final_lin_pos_ = {X, Y};
  // bounds_final_lin_vel_ = {X, Y, Z};
  // bounds_final_ang_pos_ = {X, Y, Z};
  // bounds_final_ang_vel_ = {X, Y, Z};
}

void Parameters::from_yaml(const YAML::Node &node) {
  int num_leg(2);
  ee_phase_durations_.resize(num_leg);
  ee_in_contact_at_start_.resize(num_leg);
  Eigen::VectorXd tmp_vec;
  bool tmp_bool;
  try {
    ReadParameter(node, "duration_base_polynomial", duration_base_polynomial_);
    ReadParameter(node, "force_polynomials_per_stance_phase",
                  force_polynomials_per_stance_phase_);
    ReadParameter(node, "ee_polynomials_per_swing_phase",
                  ee_polynomials_per_swing_phase_);
    ReadParameter(node, "force_limit_in_normal_direction",
                  force_limit_in_normal_direction_);
    ReadParameter(node, "dt_constraint_range_of_motion",
                  dt_constraint_range_of_motion_);
    ReadParameter(node, "dt_constraint_dynamic", dt_constraint_dynamic_);
    ReadParameter(node, "dt_constraint_base_motion",
                  dt_constraint_base_motion_);
    ReadParameter(node, "b_optimize_timings", b_optimize_timings);
    ReadParameter(node, "bound_phase_duration", tmp_vec);
    bound_phase_duration_ = std::make_pair(tmp_vec(0), tmp_vec(1));
    for (auto ee : {L, R}) {
      ReadParameter(node["ee_phase_durations"], std::to_string(ee), tmp_vec);
      for (int i = 0; i < tmp_vec.size(); ++i)
        ee_phase_durations_.at(ee).push_back(tmp_vec(i));
      ReadParameter(node["ee_in_contact_at_start"], std::to_string(ee),
                    tmp_bool);
      ee_in_contact_at_start_.at(ee) = tmp_bool;
    }

    ReadParameter(node["costs"], "w_FinalBaseLinPosCost",
                  w_FinalBaseLinPosCost);
    ReadParameter(node["costs"], "w_FinalBaseLinVelCost",
                  w_FinalBaseLinVelCost);
    ReadParameter(node["costs"], "w_FinalBaseAngPosCost",
                  w_FinalBaseAngPosCost);
    ReadParameter(node["costs"], "w_FinalBaseAngVelCost",
                  w_FinalBaseAngVelCost);
    ReadParameter(node["costs"], "w_FinalEEMotionLinPosCost",
                  w_FinalEEMotionLinPosCost);
    ReadParameter(node["costs"], "w_FinalEEMotionAngPosCost",
                  w_FinalEEMotionAngPosCost);
    ReadParameter(node["costs"], "w_IntermediateBaseLinVelCost",
                  w_IntermediateBaseLinVelCost);
    ReadParameter(node["costs"], "w_IntermediateBaseAngVelCost",
                  w_IntermediateBaseAngVelCost);
    ReadParameter(node["costs"], "w_BaseLinVelDiffCost", w_BaseLinVelDiffCost);
    ReadParameter(node["costs"], "w_BaseAngVelDiffCost", w_BaseAngVelDiffCost);
    ReadParameter(node["costs"], "w_WrenchLinPosCost", w_WrenchLinPosCost);
    ReadParameter(node["costs"], "w_WrenchLinVelCost", w_WrenchLinVelCost);
    ReadParameter(node["costs"], "w_WrenchAngPosCost", w_WrenchAngPosCost);
    ReadParameter(node["costs"], "w_WrenchAngVelCost", w_WrenchAngVelCost);
    ReadParameter(node["costs"], "w_WrenchLinVelDiffCost",
                  w_WrenchLinVelDiffCost);
    ReadParameter(node["costs"], "w_WrenchAngVelDiffCost",
                  w_WrenchAngVelDiffCost);

  } catch (std::runtime_error &e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
  }

  initialize();
}

void Parameters::OptimizePhaseDurations() { constraints_.push_back(TotalTime); }

Parameters::VecTimes Parameters::GetBasePolyDurations() const {
  std::vector<double> base_spline_timings_;
  double dt = duration_base_polynomial_;
  double t_left = GetTotalTime();

  double eps = 1e-10; // since repeated subtraction causes inaccuracies
  while (t_left > eps) {
    double duration = t_left > dt ? dt : t_left;
    base_spline_timings_.push_back(duration);

    t_left -= dt;
  }

  return base_spline_timings_;
}

int Parameters::GetPhaseCount(EEID ee) const {
  return ee_phase_durations_.at(ee).size();
}

int Parameters::GetEECount() const { return ee_in_contact_at_start_.size(); }

double Parameters::GetTotalTime() const {
  std::vector<double> T_feet;

  for (const auto &v : ee_phase_durations_)
    T_feet.push_back(std::accumulate(v.begin(), v.end(), 0.0));

  // safety check that all feet durations sum to same value
  double T =
      T_feet.empty() ? 0.0 : T_feet.front(); // take first foot as reference
  for (double Tf : T_feet)
    assert(fabs(Tf - T) < 1e-6);

  return T;
}

bool Parameters::IsOptimizeTimings() const {
  // if total time is constrained, then timings are optimized
  ConstraintName c = TotalTime;
  auto v = constraints_; // shorthand
  return std::find(v.begin(), v.end(), c) != v.end();
}

} // namespace towr_plus
