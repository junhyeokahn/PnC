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

#include <PnC/Planner/towr_plus/include/towr_plus/variables/phase_durations.h>
#include <PnC/Planner/towr_plus/include/towr_plus/variables/spline.h> // for Spline::GetSegmentID()
#include <PnC/Planner/towr_plus/include/towr_plus/variables/variable_names.h>

#include <numeric> // std::accumulate

namespace towr_plus {

PhaseDurations::PhaseDurations(EndeffectorID ee, const VecDurations &timings,
                               bool is_first_phase_in_contact,
                               double lhs_bound_buffer, double rhs_bound_buffer)
    // -1 since last phase-duration is not optimized over, but comes from total
    // time
    : VariableSet(timings.size() - 1, id::EESchedule(ee)) {
  durations_ = timings;
  t_total_ = std::accumulate(timings.begin(), timings.end(), 0.0);
  initial_contact_state_ = is_first_phase_in_contact;
  lhs_bound_buffer_ = lhs_bound_buffer;
  rhs_bound_buffer_ = rhs_bound_buffer;
}

void PhaseDurations::AddObserver(PhaseDurationsObserver *const o) {
  observers_.push_back(o);
}

void PhaseDurations::UpdateObservers() const {
  for (auto &spline : observers_)
    spline->UpdatePolynomialDurations();
}

Eigen::VectorXd PhaseDurations::GetValues() const {
  VectorXd x(GetRows());

  for (int i = 0; i < x.rows(); ++i)
    x(i) = durations_.at(i);

  return x;
}

void PhaseDurations::SetVariables(const VectorXd &x) {
  // the sum of all phase durations should never be larger than the total time
  // of the trajectory. This would e.g. query constraints after duration of
  // the trajectory and causes undefined behavior. However, when IPOPT
  // optimizes the phase durations, it's not possible to enforce that in every
  // iteration this condition is fulfilled. Sure, we can set this as a
  // constraint, but during the solution process constraints might still be
  // violated. Fortunately, violation of this doesn't seem to mess up IPOPT
  // too much and a solution is often found. So if you get this error you can
  // ignore it by compiling in Release mode, but I'm leaving this in here to
  // show that this is undefined behavior and a clean implementation is still
  // required. PR desired ;)
  assert(t_total_ > x.sum());

  for (int i = 0; i < GetRows(); ++i)
    durations_.at(i) = x(i);

  // last phase duration not optimized, used to fill up to total time.
  durations_.back() = t_total_ - x.sum();
  UpdateObservers();
}

PhaseDurations::VecBound PhaseDurations::GetBounds() const {
  VecBound bounds;

  for (int i = 0; i < GetRows(); ++i) {
    ifopt::Bounds b = ifopt::Bounds(durations_[i] - lhs_bound_buffer_,
                                    durations_[i] + rhs_bound_buffer_);
    bounds.push_back(b);
  }

  return bounds;
}

PhaseDurations::VecDurations PhaseDurations::GetPhaseDurations() const {
  return durations_;
}

bool PhaseDurations::IsContactPhase(double t) const {
  int phase_id = Spline::GetSegmentID(t, durations_);
  return phase_id % 2 == 0 ? initial_contact_state_ : !initial_contact_state_;
}

PhaseDurations::Jacobian
PhaseDurations::GetJacobianOfPos(int current_phase, const VectorXd &dx_dT,
                                 const VectorXd &xd) const {
  int n_dim = xd.rows();
  Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(n_dim, GetRows());

  bool in_last_phase = (current_phase == durations_.size() - 1);

  // duration of current phase expands and compressed spline
  if (!in_last_phase)
    jac.col(current_phase) = dx_dT;

  for (int phase = 0; phase < current_phase; ++phase) {
    // each previous durations shifts spline along time axis
    jac.col(phase) = -1 * xd;

    // in last phase previous duration cause expansion/compression of spline
    // as final time is fixed.
    if (in_last_phase)
      jac.col(phase) -= dx_dT;
  }

  // convert to sparse, but also regard 0.0 as non-zero element, because
  // could turn nonzero during the course of the program
  // as durations change and t_global falls into different spline
  return jac.sparseView(1.0, -1.0);
}

} /* namespace towr_plus */
