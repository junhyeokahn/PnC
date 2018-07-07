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
 */

#include <solver/optimizer/SparseCholesky.hpp>

namespace linalg {

  void SparseCholesky::analyzePattern(const Eigen::SparseMatrix<double>& mat, const solver::SolverSetting& stgs)
  {
	n_ = mat.cols();
	X_.resize(n_);
	D_.resize(n_);
	Y_.resize(n_);
	Flag_.resize(n_);
	Lnnz_.resize(n_);
	L_.resize(n_,n_);
	Parent_.resize(n_);
	Pattern_.resize(n_);
	std::vector<Eigen::Triplet<double>> coeffs;
	stgs_ = std::make_shared<solver::SolverSetting>(stgs);

	int* Lnnz = Lnnz_.data();
	int* Flag = Flag_.data();
	int* Parent = Parent_.data();

	for (int k=0; k<mat.outerSize(); k++) {
      Flag[k] = k;
      Lnnz[k] = 0;
      Parent[k] = -1;
      for (Eigen::SparseMatrix<double>::InnerIterator it(mat,k); it; ++it)
		for (int row=it.row(); Flag[row]!=k ; row=Parent[row]){
		  if (Parent[row]==-1) { Parent[row]=k; }
		  Lnnz[row]++ ;
		  Flag[row] = k;
		}
    }

	for (int col=0; col<n_; col++)
	  for (int row=0; row<Lnnz[col]; row++)
	    coeffs.push_back(Eigen::Triplet<double>(int(row%n_),col,1.0));
	L_.setFromTriplets(coeffs.begin(), coeffs.end());
	if (!L_.isCompressed()) { L_.makeCompressed(); }
  }

  int SparseCholesky::factorize(const Eigen::SparseMatrix<double>& mat, const Eigen::Ref<const Eigen::VectorXd>& sign)
  {
	double* D = D_.data();
	double* Y = Y_.data();
	int* Lnnz = Lnnz_.data();
	int* Flag = Flag_.data();
	int* Parent = Parent_.data();
	int* Pattern = Pattern_.data();

	double* Lx = L_.valuePtr();
	int* Lp = L_.outerIndexPtr();
	int* Li = L_.innerIndexPtr();

	const double* Ax = mat.valuePtr();
	const int* Ap = mat.outerIndexPtr();
	const int* Ai = mat.innerIndexPtr();

	int p, len;
	delta_ = stgs_->get(solver::SolverDoubleParam_DynamicRegularization);
	eps_ = stgs_->get(solver::SolverDoubleParam_DynamicRegularizationThresh);

	for (int k=0; k<mat.outerSize(); k++) {
      // nonzero pattern of kth row of L
      Y[k] = 0.0;
      Lnnz[k] = 0;
      Flag[k] = k;
      int top = n_;
	  for (int p=Ap[k]; p<Ap[k+1]; p++) {
		int row = Ai[p];
		Y[row]  = Ax[p];
		for (len=0; Flag[row]!=k; row=Parent[row]) {
		  Pattern[len++] = row;
		  Flag[row] = k;
		}
		while (len>0) Pattern[--top] = Pattern[--len] ;
	  }

	  // numerical values kth row of L
	  D[k] = Y[k];
	  Y[k] = 0.0;
	  for (; top<n_; top++) {
		double yi = Y[Pattern[top]];
		Y[Pattern[top]] = 0.0;
		for (p=Lp[Pattern[top]]; p<Lp[Pattern[top]]+Lnnz[Pattern[top]]; p++)
		  Y[Li[p]] -= Lx[p]*yi;
		double l_ki = yi/D[Pattern[top]];
		D[k] -= l_ki*yi;
		Li [p] = k;
		Lx [p] = l_ki;
		Lnnz[Pattern[top]]++;
	  }

	  // Dynamic regularization
      D[k] = sign[k]*D[k] <= eps_ ? sign[k]*delta_ : D[k];
    }
    return (n_) ;
}

  void SparseCholesky::solve( const Eigen::Ref<const Eigen::VectorXd>& b, double* x)
  {
	double* D = D_.data();
	double* Lx = L_.valuePtr();
	int* Lp = L_.outerIndexPtr();
	int* Li = L_.innerIndexPtr();

	int n = b.size();
	Eigen::Map<Eigen::VectorXd> eig_x(x, b.size());
	eig_x = b;
    for (int j=0; j<n ; j++)
	  for (int p=Lp[j]; p<Lp[j+1]; p++)
		x[Li[p]] -= Lx[p]*x[j];

    for (int j=0; j<n; j++){ x[j] /= D[j]; }

    for (int j=n-1; j>=0; j--)
	  for (int p=Lp[j]; p<Lp[j+1]; p++)
		x[j] -= Lx[p] * x[Li[p]];
  }

  Eigen::VectorXd& SparseCholesky::solve(const Eigen::VectorXd& b)
  {
	X_ = b;
	for (int k=0; k<L_.outerSize(); k++)
	  for (Eigen::SparseMatrix<double>::InnerIterator it(L_,k); it; ++it)
		X_[it.row()] -= it.value()*X_[it.col()];

    X_.array() /= D_.array();
	for (int k=L_.outerSize()-1; k>=0; k--)
	  for (Eigen::SparseMatrix<double>::InnerIterator it(L_,k); it; ++it)
		X_[it.col()] -= it.value()*X_[it.row()];

    return X_;
  }

}
