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

#include <solver/interface/Cone.hpp>
#include <solver/optimizer/LinSolver.hpp>
#include <solver/optimizer/EqRoutine.hpp>
#include <solver/optimizer/InfoPrinter.hpp>
#include <solver/interface/SolverSetting.hpp>

namespace solver {

  /**
   * Main class that implements an Interior Point Solver for Second-Order Cones.
   * Details can be found in the paper: Domahidi, A. and Chu, E. and Boyd, S.,
   *     ECOS: An SOCP solver for embedded systems, ECC 2013, pages 3071-3076
   */
  class InteriorPointSolver
  {
    public:
	  InteriorPointSolver(){}
	  ~InteriorPointSolver() {}

	  void internalInitialization();
	  void initialize(SolverStorage& stg, Cone& cone, SolverSetting& stgs);
	  ExitCode optimize();

	  Cone& getCone() { return *cone_; }
	  SolverStorage& getStg() { return *stg_; }
      SolverSetting& getStgs() { return *stgs_; }
	  EqRoutine& getEqRoutine() { return equil_; }
	  OptimizationInfo& getInfo() { return info_; }
      InfoPrinter& getPrinter() { return printer_; }
      LinSolver& getLinSolver() { return linsolver_; }
	  OptimizationInfo& getBestInfo() { return best_info_; }

	  const Cone& getCone() const { return *cone_; }
	  const SolverStorage& getStg() const { return *stg_; }
      const SolverSetting& getStgs() const { return *stgs_; }
	  const EqRoutine& getEqRoutine() const { return equil_; }
	  const OptimizationInfo& getInfo() const { return info_; }
	  const InfoPrinter& getPrinter() const { return printer_; }
	  const LinSolver& getLinSolver() const { return linsolver_; }
	  const OptimizationInfo& getBestInfo() const { return best_info_; }

	  OptimizationVector& optsol() { return opt_; }
	  const OptimizationVector& optsol() const { return opt_; }

    private:
	  void rhsAffineStep();
	  void saveIterateAsBest();
	  void restoreBestIterate();
	  void rhsCenteringPredictorStep();
	  ExitCode convergenceCheck(const PrecisionConvergence& mode);

	  void computeResiduals();
	  void updateStatistics();
	  ExitCode initializeVariables();
	  double lineSearch(const Eigen::Ref<const Eigen::VectorXd>& dsvec, const Eigen::Ref<const Eigen::VectorXd>& dzvec, double tau, double dtau, double kappa, double dkappa);

    private:
	  Vector res_;
      ExitCode exitcode_;
      ExtendedVector RHS1_, RHS2_;
      OptimizationVector opt_, best_opt_, dopt1_, dopt2_;
      ConicVector lambda_, rho_, sigma_, lbar_, ds_affine_by_W_, W_times_dz_affine_, ds_combined_, dz_combined_;
	  double dk_combined_, dt_affine_, dk_affine_, inires_x_, inires_y_, inires_z_, dt_denom_,
	         residual_t_, residual_x_, residual_y_, residual_z_, cx_, by_, hz_, prev_pres_;

	  Cone* cone_;
	  EqRoutine equil_;
	  SolverStorage* stg_;
	  SolverSetting* stgs_;
	  LinSolver linsolver_;
	  InfoPrinter printer_;
	  OptimizationInfo info_, best_info_;
  };
}
