/*
 *
 * LDL Copyright (c) 2005-2012 by Timothy A. Davis. http://www.suitesparse.com
 *
 * LDL License:
 *
 *    Your use or distribution of LDL or any modified version of
 *    LDL implies that you agree to this License.
 *
 *    This library is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 2.1 of the License, or (at your option) any later version.
 *
 *    This library is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with this library; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301
 *    USA
 *
 *    Permission is hereby granted to use or copy this program under the
 *    terms of the GNU LGPL, provided that the Copyright, this License,
 *    and the Availability of the original version is retained on all copies.
 *    User documentation of any code that uses this code or any modified
 *    version of this code must cite the Copyright, this License, the
 *    Availability note, and "Used by permission." Permission to modify
 *    the code and to distribute modified code is granted, provided the
 *    Copyright, this License, and the Availability note are retained,
 *    and a notice that the code was modified is included.
 *
 * Availability:
 *
 *    http://www.suitesparse.com
 *
 * Stripped down by Alexander Domahidi, 2012.
 * Modified to c++ code by Max Planck Society, 2017.
 *
 */

#pragma once

#include <memory>
#include <Eigen/Sparse>
#include <solver/interface/SolverSetting.hpp>

namespace linalg {

  /**
   * Class to perform an LDL factorization of a matrix
   */
  class SparseCholesky
  {
    public:
	  SparseCholesky(){}
	  ~SparseCholesky(){}

	  void analyzePattern(const Eigen::SparseMatrix<double>& mat, const solver::SolverSetting& stgs);
	  int  factorize(const Eigen::SparseMatrix<double>& mat, const Eigen::Ref<const Eigen::VectorXd>& sign);
	  void solve(const Eigen::Ref<const Eigen::VectorXd>& b, double* x);
	  Eigen::VectorXd& solve(const Eigen::VectorXd& b);

    private:
	  int n_;
	  double eps_, delta_;
	  Eigen::VectorXd D_, Y_, X_;
	  Eigen::SparseMatrix<double> L_;
	  std::shared_ptr<solver::SolverSetting> stgs_;
	  Eigen::VectorXi Parent_, Pattern_, Flag_, Lnnz_;
  };

}
