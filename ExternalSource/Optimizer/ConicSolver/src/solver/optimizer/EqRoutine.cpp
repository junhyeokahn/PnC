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

#include <solver/optimizer/EqRoutine.hpp>

namespace solver {

  void EqRoutine::setEquilibration(const Cone& cone, const SolverSetting& stgs, SolverStorage& stg)
  {
	equil_vec_.initialize(cone);
	stgs_ = std::make_shared<SolverSetting>(stgs);
	cone_ = std::make_shared<Cone>(cone);
	this->ruizEquilibration(stg);
  }

  void EqRoutine::ruizEquilibration(SolverStorage& stg)
  {
    Vector equil_tmp;
    equil_vec_.setOnes();
    equil_tmp.initialize(*cone_);

    // iterative equilibration
    for (int iter=0; iter<stgs_->get(SolverIntParam_EquilibrationIters); iter++) {
      equil_tmp.setZero();

      // infinity norms of rows and columns of optimization matrices
      if (stg.A().nonZeros()>0) { maxRowsCols(equil_tmp.y().data(), equil_tmp.x().data(), stg.A()); }
      if (stg.G().nonZeros()>0) { maxRowsCols(equil_tmp.z().data(), equil_tmp.x().data(), stg.G()); }

      // equilibration of second order cones
      for (int i=0; i<cone_->numSoc(); i++) { equil_tmp.zSoc(i).setConstant( equil_tmp.zSoc(i).sum() ); }
      for (int i=0; i<cone_->sizeProb(); i++) { equil_tmp[i] = fabs(equil_tmp[i]) < 1e-6 ? 1.0 : sqrt(equil_tmp[i]); }

      // matrices equilibration
      if (stg.A().nonZeros()>0) { equilibrateRowsCols(equil_tmp.y().data(), equil_tmp.x().data(), stg.A()); }
      if (stg.G().nonZeros()>0) { equilibrateRowsCols(equil_tmp.z().data(), equil_tmp.x().data(), stg.G()); }

      // update equilibration vector
      equil_vec_.array() *= equil_tmp.array();
    }
    stg.cbh().array() /= equil_vec_.array();
  }

  void EqRoutine::maxRowsCols(double *row_vec, double *col_vec, const Eigen::SparseMatrix<double>& mat)
  {
	const int* outPtr = mat.outerIndexPtr();
    for (int col=0; col<mat.cols(); col++) {
      if (outPtr[col+1]-outPtr[col]>0) {
  	    Eigen::Map<const Eigen::VectorXd> eig_col(&mat.valuePtr()[outPtr[col]], outPtr[col+1]-outPtr[col]);
  	    col_vec[col] = eig_col.cwiseAbs().maxCoeff();
  	    for (Eigen::SparseMatrix<double>::InnerIterator it(mat,col); it; ++it)
  	      row_vec[it.row()] = std::max(fabs(it.value()), row_vec[it.row()]);
      }
    }
  }

  void EqRoutine::equilibrateRowsCols(const double *row_vec, const double *col_vec, Eigen::SparseMatrix<double>& mat)
  {
	for (int k=0; k<mat.outerSize(); ++k)
	  for (Eigen::SparseMatrix<double>::InnerIterator it(mat,k); it; ++it)
	    it.valueRef() /= (col_vec[it.col()] * row_vec[it.row()]);
  }

  void EqRoutine::unsetEquilibration(SolverStorage& stg)
  {
	if (stg.A().nonZeros()>0) { unequilibrateRowsCols(equil_vec_.y().data(), equil_vec_.x().data(), stg.A()); }
	if (stg.G().nonZeros()>0) { unequilibrateRowsCols(equil_vec_.z().data(), equil_vec_.x().data(), stg.G()); }
    stg.cbh().array() *= equil_vec_.array();
  }

  void EqRoutine::unequilibrateRowsCols(const double *row_vec, const double *col_vec, Eigen::SparseMatrix<double>& mat)
  {
	for (int k=0; k<mat.outerSize(); ++k)
	  for (Eigen::SparseMatrix<double>::InnerIterator it(mat,k); it; ++it)
	    it.valueRef() *= (row_vec[it.row()] * col_vec[it.col()]);
  }

  void EqRoutine::scaleVariables(OptimizationVector& opt)
  {
    opt.xyz().array() /= (equil_vec_*opt.tau()).array();
    opt.s().array() *= (equil_vec_.z()/opt.tau()).array();
  }

}
