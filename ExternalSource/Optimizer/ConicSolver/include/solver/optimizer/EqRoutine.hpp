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

#include <memory>
#include <Eigen/Sparse>
#include <solver/interface/Cone.hpp>
#include <solver/interface/SolverSetting.hpp>

namespace solver {

  /*! Equilibration routine to improve condition number of
   *  matrices involved in the optimization problem. The method
   *  provided by default is Ruiz equilibration.
   */
  class EqRoutine
  {
    public:
	  EqRoutine(){}
	  ~EqRoutine(){}

	  void setEquilibration(const Cone& cone, const SolverSetting& stgs, SolverStorage& stg);
	  void unsetEquilibration(SolverStorage& stg);
	  void scaleVariables(OptimizationVector& opt);

    private:
	  void ruizEquilibration(SolverStorage& stg);
	  void maxRowsCols(double *row_vec, double *col_vec, const Eigen::SparseMatrix<double>& mat);
	  void equilibrateRowsCols(const double *row_vec, const double *col_vec, Eigen::SparseMatrix<double>& mat);
	  void unequilibrateRowsCols(const double *row_vec, const double *col_vec, Eigen::SparseMatrix<double>& mat);

    private:
	  Vector equil_vec_;
	  std::shared_ptr<Cone> cone_;
	  std::shared_ptr<SolverSetting> stgs_;
  };

}
