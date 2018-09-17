/*
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

namespace solver {

    /*! Available integer variables used by the optimizer */
  enum SolverIntParam {
	// Equilibration parameters
	SolverIntParam_EquilibrationIters,

	// Linear System parameters
	SolverIntParam_NumIterRefinementsLinSolve,

	// Model parameters
	SolverIntParam_MaxIters,
    SolverIntParam_WarmStartIters,
    SolverIntParam_SolverMaxIters,
	SolverIntParam_NumberRefinementsTrustRegion,

	// Variable parameters
    SolverIntParam_ColNum,

	// OptimizationInfo parameters
	SolverIntParam_NumIter,
	SolverIntParam_NumRefsLinSolve,
	SolverIntParam_NumRefsLinSolveAffine,
	SolverIntParam_NumRefsLinSolveCorrector,
  };

  /*! Available boolean variables used by the optimizer */
  enum SolverBoolParam {
	SolverBoolParam_Verbose,
  };

  /*! Available double variables used by the optimizer */
  enum SolverDoubleParam {
	// Convergence tolerances
	SolverDoubleParam_FeasibilityTol,
	SolverDoubleParam_DualityGapAbsTol,
	SolverDoubleParam_DualityGapRelTol,
	SolverDoubleParam_FeasibilityTolInacc,
	SolverDoubleParam_DualityGapAbsTolInacc,
	SolverDoubleParam_DualityGapRelTolInacc,

	// Linear System parameters
	SolverDoubleParam_LinearSystemAccuracy,
    SolverDoubleParam_ErrorReductionFactor,
	SolverDoubleParam_StaticRegularization,
	SolverDoubleParam_DynamicRegularization,
	SolverDoubleParam_DynamicRegularizationThresh,

	// Algorithm parameters
	SolverDoubleParam_SafeGuard,
	SolverDoubleParam_MinimumStepLength,
	SolverDoubleParam_MaximumStepLength,
	SolverDoubleParam_StepLengthScaling,
	SolverDoubleParam_MinimumCenteringStep,
	SolverDoubleParam_MaximumCenteringStep,

	// Model parameters
	SolverDoubleParam_TrustRegionThreshold,
	SolverDoubleParam_SoftConstraintWeight,

	// Variable parameters
    SolverDoubleParam_X,
    SolverDoubleParam_LB,
    SolverDoubleParam_UB,
    SolverDoubleParam_Guess,

	// OptimizationInfo parameters
	SolverDoubleParam_Tau,
	SolverDoubleParam_Kappa,
	SolverDoubleParam_DualCost,
	SolverDoubleParam_PrimalCost,
	SolverDoubleParam_DualityGap,
	SolverDoubleParam_KappaOverTau,
	SolverDoubleParam_DualResidual,
	SolverDoubleParam_MeritFunction,
	SolverDoubleParam_PrimalResidual,
	SolverDoubleParam_DualInfeasibility,
	SolverDoubleParam_RelativeDualityGap,
	SolverDoubleParam_PrimalInfeasibility,

	SolverDoubleParam_StepLength,
	SolverDoubleParam_AffineStepLength,
	SolverDoubleParam_CorrectionStepLength,
  };

}
