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

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <solver/interface/SolverSetting.hpp>

namespace solver {

  void SolverSetting::initialize(const std::string cfg_file, const std::string stgs_vars_yaml)
  {
    try
    {
	  YAML::Node stgs_cfg = YAML::LoadFile(cfg_file.c_str());
	  YAML::Node stgs_vars = stgs_cfg[stgs_vars_yaml.c_str()];

	  // Convergence tolerances
	  feasibility_tolerance_ = stgs_vars["feasibility_tolerance"].as<double>();
	  absolute_suboptimality_gap_ = stgs_vars["absolute_suboptimality_gap"].as<double>();
	  relative_suboptimality_gap_ = stgs_vars["relative_suboptimality_gap"].as<double>();
	  feasibility_tolerance_inaccurate_ = stgs_vars["feasibility_tolerance_inaccurate"].as<double>();
	  absolute_suboptimality_gap_inaccurate_ = stgs_vars["absolute_suboptimality_gap_inaccurate"].as<double>();
	  relative_suboptimality_gap_inaccurate_ = stgs_vars["relative_suboptimality_gap_inaccurate"].as<double>();

	  // Equilibration parameters
	  equil_iterations_ = stgs_vars["equil_iterations"].as<int>();

	  // Linear System parameters
	  dyn_reg_thresh_ = stgs_vars["dyn_reg_thresh"].as<double>();
	  lin_sys_accuracy_ = stgs_vars["lin_sys_accuracy"].as<double>();
	  err_reduction_factor_ = stgs_vars["err_reduction_factor"].as<double>();
	  num_iter_ref_lin_solve_ = stgs_vars["num_iter_ref_lin_solve"].as<int>();
	  static_regularization_ = stgs_vars["static_regularization"].as<double>();
	  dynamic_regularization_ = stgs_vars["dynamic_regularization"].as<double>();

      // Algorithm parameters
	  safeguard_ = stgs_vars["safeguard"].as<double>();
	  min_step_length_ = stgs_vars["min_step_length"].as<double>();
	  max_step_length_ = stgs_vars["max_step_length"].as<double>();
	  min_centering_step_ = stgs_vars["min_centering_step"].as<double>();
	  max_centering_step_ = stgs_vars["max_centering_step"].as<double>();
	  step_length_scaling_ = stgs_vars["step_length_scaling"].as<double>();

	  // Model parameters
	  verbose_ = stgs_vars["verbose"].as<bool>();

	  max_iters_ = stgs_vars["max_iters"].as<int>();
	  ipsolver_max_iters_ = stgs_vars["ipsolver_max_iters"].as<int>();
	  ipsolver_warm_iters_ = stgs_vars["ipsolver_warm_iters"].as<int>();

	  num_itrefs_trustregion_ = stgs_vars["num_itrefs_trustregion"].as<int>();
	  trust_region_threshold_ = stgs_vars["trust_region_threshold"].as<double>();
	  soft_constraint_weight_ = stgs_vars["soft_constraint_weight"].as<double>();
    }
    catch (YAML::ParserException &e)
    {
      std::cout << e.what() << "\n";
    }
  }

  // getter and setter methods for boolean parameters
  bool SolverSetting::get(SolverBoolParam param) const
  {
    switch (param)
    {
      // Model parameters
      case SolverBoolParam_Verbose: { return verbose_; }

      // Not handled parameters
      default: { throw std::runtime_error("SolverSetting::get SolverBoolParam invalid"); break; }
    }
  }

  void SolverSetting::set(SolverBoolParam param, bool value)
  {
    switch (param)
    {
      // Model parameters
      case SolverBoolParam_Verbose: { verbose_ = value; break; }

      // Not handled parameters
      default: { throw std::runtime_error("SolverSetting::set SolverBoolParam invalid"); break; }
    }
  }

  // getter and setter methods for integer parameters
  int SolverSetting::get(SolverIntParam param) const
  {
    switch (param)
    {
      // Equilibration parameters
      case SolverIntParam_EquilibrationIters : { return equil_iterations_; }

      // Linear System parameters
      case SolverIntParam_NumIterRefinementsLinSolve : { return num_iter_ref_lin_solve_; }

      // Model parameters
      case SolverIntParam_MaxIters: { return max_iters_; }
      case SolverIntParam_SolverMaxIters : { return ipsolver_max_iters_; }
      case SolverIntParam_WarmStartIters : { return ipsolver_warm_iters_; }
      case SolverIntParam_NumberRefinementsTrustRegion : { return num_itrefs_trustregion_; }
      default: { throw std::runtime_error("SolverSetting::get SolverIntParam invalid"); break; }
    }
  }

  void SolverSetting::set(SolverIntParam param, int value)
  {
    switch (param)
    {
      // Equilibration parameters
      case SolverIntParam_EquilibrationIters : { equil_iterations_ = value; break; }

      // Linear System parameters
      case SolverIntParam_NumIterRefinementsLinSolve : { num_iter_ref_lin_solve_ = value; break; }

      // Model parameters
      case SolverIntParam_MaxIters : { max_iters_ = value; break; }
      case SolverIntParam_SolverMaxIters : { ipsolver_max_iters_ = value; break; }
      case SolverIntParam_WarmStartIters : { ipsolver_warm_iters_ = value; break; }
      case SolverIntParam_NumberRefinementsTrustRegion : { num_itrefs_trustregion_ = value; break; }
      default: { throw std::runtime_error("SolverSetting::set SolverIntParam invalid"); break; }
    }
  }

  // getter and setter methods for double parameters
  double SolverSetting::get(SolverDoubleParam param) const
  {
    switch (param)
    {
      // Convergence tolerances
      case SolverDoubleParam_FeasibilityTol: { return feasibility_tolerance_; }
      case SolverDoubleParam_DualityGapAbsTol: { return absolute_suboptimality_gap_; }
      case SolverDoubleParam_DualityGapRelTol: { return relative_suboptimality_gap_; }
      case SolverDoubleParam_FeasibilityTolInacc: { return feasibility_tolerance_inaccurate_; }
      case SolverDoubleParam_DualityGapAbsTolInacc: { return absolute_suboptimality_gap_inaccurate_; }
      case SolverDoubleParam_DualityGapRelTolInacc: { return relative_suboptimality_gap_inaccurate_; }

      // Linear System parameters
      case SolverDoubleParam_LinearSystemAccuracy : { return lin_sys_accuracy_; }
      case SolverDoubleParam_ErrorReductionFactor : { return err_reduction_factor_; }
      case SolverDoubleParam_DynamicRegularizationThresh : { return dyn_reg_thresh_; }
      case SolverDoubleParam_StaticRegularization : { return static_regularization_; }
      case SolverDoubleParam_DynamicRegularization : { return dynamic_regularization_; }

      // Algorithm parameters
      case SolverDoubleParam_SafeGuard : { return safeguard_; }
      case SolverDoubleParam_MinimumStepLength : { return min_step_length_; }
      case SolverDoubleParam_MaximumStepLength : { return max_step_length_; }
      case SolverDoubleParam_StepLengthScaling : { return step_length_scaling_; }
      case SolverDoubleParam_MinimumCenteringStep : { return min_centering_step_; }
      case SolverDoubleParam_MaximumCenteringStep : { return max_centering_step_; }

      // Model parameters
      case SolverDoubleParam_TrustRegionThreshold : { return trust_region_threshold_; }
      case SolverDoubleParam_SoftConstraintWeight : { return soft_constraint_weight_; }

      // Not handled parameters
      default: { throw std::runtime_error("SolverSetting::get SolverDoubleParam invalid"); break; }
    }
  }

  void SolverSetting::set(SolverDoubleParam param, double value)
  {
    switch (param)
    {
      // Convergence tolerances
      case SolverDoubleParam_FeasibilityTol: { feasibility_tolerance_ = value; break; }
      case SolverDoubleParam_DualityGapAbsTol: { absolute_suboptimality_gap_ = value; break; }
      case SolverDoubleParam_DualityGapRelTol: { relative_suboptimality_gap_ = value; break; }
      case SolverDoubleParam_FeasibilityTolInacc: { feasibility_tolerance_inaccurate_ = value; break; }
      case SolverDoubleParam_DualityGapAbsTolInacc: { absolute_suboptimality_gap_inaccurate_ = value; break; }
      case SolverDoubleParam_DualityGapRelTolInacc: { relative_suboptimality_gap_inaccurate_ = value; break; }

      // Linear System parameters
      case SolverDoubleParam_LinearSystemAccuracy : { lin_sys_accuracy_ = value; break; }
      case SolverDoubleParam_ErrorReductionFactor : { err_reduction_factor_ = value; break; }
      case SolverDoubleParam_DynamicRegularizationThresh : { dyn_reg_thresh_ = value; break; }
      case SolverDoubleParam_StaticRegularization : { static_regularization_ = value; break; }
      case SolverDoubleParam_DynamicRegularization : { dynamic_regularization_ = value; break; }

      // Algorithm parameters
      case SolverDoubleParam_SafeGuard : { safeguard_ = value; break; }
      case SolverDoubleParam_MinimumStepLength : { min_step_length_ = value; break; }
      case SolverDoubleParam_MaximumStepLength : { max_step_length_ = value; break; }
      case SolverDoubleParam_StepLengthScaling : { step_length_scaling_ = value; break; }
      case SolverDoubleParam_MinimumCenteringStep : { min_centering_step_ = value; break; }
      case SolverDoubleParam_MaximumCenteringStep : { max_centering_step_ = value; break; }

      // Model parameters
      case SolverDoubleParam_TrustRegionThreshold : { trust_region_threshold_ = value; break; }
      case SolverDoubleParam_SoftConstraintWeight : { soft_constraint_weight_ = value; break; }

      // Not handled parameters
      default: { throw std::runtime_error("SolverSetting::set SolverDoubleParam invalid"); break; }
    }
  }

}
