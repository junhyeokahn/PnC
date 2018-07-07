/*
 *
 * ECOS - Embedded Conic Solver
 * Copyright (C) [2012-2015] A. Domahidi [domahidi@embotech.com],
 * Automatic Control Lab, ETH Zurich & embotech GmbH, Zurich, Switzerland.
 *
 * Copyright [2017] Max Planck Society. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <string>
#include <limits>
#include <solver/interface/SolverParams.hpp>

namespace solver {

  enum class ExitCode {
    Optimal,           /*! Problem solved to optimality                           */
    OptimalInacc,      /*! Problem solved to optimality (Inaccurate)              */
    PrimalInf,         /*! Found certificate of primal infeasibility              */
    PrimalInfInacc,    /*! Found certificate of primal infeasibility (Inaccurate) */
    DualInf,           /*! Found certificate of dual infeasibility                */
    DualInfInacc,      /*! Found certificate of dual infeasibility (Inaccurate)   */
	ReachMaxIters,     /*! Algorithm reached maximum number of iterations         */
    NotConverged,      /*! Algorithm has not converged yet                        */
    Indeterminate,     /*! Nothing can be said about the solution                 */
    PrSearchDirection, /*! Not reliable search direction                          */
	PrSlacksLeaveCone, /*! Slack variables lie outside convex cone                */
	PrProjection,      /*! Failed in the projection of cone or linear system      */
  };

  enum class ConeStatus { Inside, Outside };
  enum class FactStatus { Optimal, Failure };
  enum class PrecisionConvergence { Full, Reduced };
  enum class QuadConstrApprox { None, TrustRegion, SoftConstraint  };

  /**
   * Class that provides access to all environment variables required by the solver
   */
  class SolverSetting
  {
    public:
	  SolverSetting(){}
	  ~SolverSetting(){}

	  void initialize(const std::string cfg_file, const std::string stgs_vars_yaml = "solver_variables");

      int get(SolverIntParam param) const;
      bool get(SolverBoolParam param) const;
      double get(SolverDoubleParam param) const;

      void set(SolverIntParam param, int value);
      void set(SolverBoolParam param, bool value);
      void set(SolverDoubleParam param, double value);

	  static constexpr double nan = ((double)0x7ff8000000000000);
	  static constexpr double inf = ((double)std::numeric_limits<double>::infinity());

    private:
	  // Convergence tolerances
	  double feasibility_tolerance_, absolute_suboptimality_gap_, relative_suboptimality_gap_,
	         feasibility_tolerance_inaccurate_, absolute_suboptimality_gap_inaccurate_, relative_suboptimality_gap_inaccurate_;

	  // Equilibration parameters
	  int equil_iterations_;

	  // Linear System parameters
	  int num_iter_ref_lin_solve_;
	  double dyn_reg_thresh_, lin_sys_accuracy_, err_reduction_factor_, static_regularization_, dynamic_regularization_;

	  // Algorithm parameters
	  double safeguard_, min_step_length_, max_step_length_, min_centering_step_, max_centering_step_, step_length_scaling_;

	  // Model parameters
	  bool verbose_;
	  double trust_region_threshold_, soft_constraint_weight_;
	  int max_iters_, num_itrefs_trustregion_, ipsolver_warm_iters_, ipsolver_max_iters_;

  };

}
