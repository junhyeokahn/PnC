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
#include <solver/interface/SolverSetting.hpp>
#include <solver/optimizer/SparseCholesky.hpp>

namespace solver {

  /**
   * Class that provides functionality for handling solution of linear systems
   * in optimization problems. Performs symbolic and numeric factorization of
   * kkt matrix, find permutation of kkt matrix to induce the best possible
   * sparsity pattern, build and updates kkt matrix and its scalings as required.
   */
  class LinSolver
  {
    public:
	  LinSolver(){}
	  ~LinSolver(){}

	  void updateMatrix();
	  void initializeMatrix();
      FactStatus numericFactorization();
      void initialize(Cone& cone, SolverSetting& stgs, SolverStorage& stg);
      int solve(const Eigen::Ref<const Eigen::VectorXd>& permB, OptimizationVector& searchDir, bool is_initialization = false);
      void matrixTransposeTimesVector(const Eigen::SparseMatrix<double>& A,const Eigen::Ref<const Eigen::VectorXd>& eig_x, Eigen::Ref<Eigen::VectorXd> eig_y, bool add = true, bool is_new = true);

	  // Some getter and setter methods
	  int perm(int id) { return perm_.indices()[id]; }
	  int invPerm(int id) { return invPerm_.indices()[id]; }
	  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>& perm() { return perm_; }
	  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>& invPerm() { return invPerm_; }
	  const Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>& perm() const { return perm_; }
	  const Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>& invPerm() const { return invPerm_; }

    private:
	  SolverSetting& getStgs() { return *stgs_; }
	  Cone& getCone() { return *cone_; }
	  SolverStorage& getStg() { return *stg_; }
	  linalg::SparseCholesky& getCholesky() { return chol_; }

	  const Cone& getCone() const { return *cone_; }
	  const SolverSetting& getStgs() const { return *stgs_; }
	  const SolverStorage& getStg() const { return *stg_; }
	  const linalg::SparseCholesky& getCholesky() const { return chol_; }

      void buildProblem();
	  void findPermutation();
	  void resizeProblemData();
	  void symbolicFactorization();

    private:
	  Cone* cone_;
	  SolverStorage* stg_;
	  SolverSetting* stgs_;

	  ConicVector Gdx_;
	  linalg::SparseCholesky chol_;
	  double static_regularization_;
	  ExtendedVector sign_, permSign_;
	  Eigen::VectorXd permX_, Pe_, permdX_;
	  Eigen::SparseMatrix<double> kkt_, permKkt_;
	  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> perm_, invPerm_, permK_;
  };

}
