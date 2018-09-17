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

#include <iostream>
#include <Eigen/Eigen>

namespace solver {

  /** Class that contains all information about the optimization variables
   *  such as upper and lower bounds, initial guess if any, and some
   *  auxiliary functions for easier handling.
   */
  class OptimizationVariable
  {
    public:
      typedef Eigen::Matrix<double, Eigen::Dynamic,              1, Eigen::ColMajor, Eigen::Dynamic, 1> OptVector;
      typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor, Eigen::Dynamic, Eigen::Dynamic> OptMatrix;

    public:
      //! Class constructor and destructor
      OptimizationVariable() : guessValueInitialized_(false) {}
      ~OptimizationVariable() {}

      //! Some alternative initialization functions
      void initialize(const char& type, int rows, int cols, double lBnd, double uBnd, int& startIndexInOptVec );
      void initialize(const char& type, int rows, int cols, OptVector& lBnd, OptVector& uBnd, int& startIndexInOptVec );
      void initialize(const char& type, int rows, int cols, OptMatrix& lBnd, OptMatrix& uBnd, int& startIndexInOptVec );
      void initialize(const char& type, int rows, int cols, double lBnd, double uBnd, int& startIndexInOptVec, double guess );
      void initialize(const char& type, int rows, int cols, double lBnd, double uBnd, int& startIndexInOptVec, OptMatrix& guess );
      void initialize(const char& type, int rows, int cols, OptVector& lBnd, OptVector& uBnd, int& startIndexInOptVec, OptMatrix& guess );
      void initialize(const char& type, int rows, int cols, OptMatrix& lBnd, OptMatrix& uBnd, int& startIndexInOptVec, OptMatrix& guess );

      //! Some helper functions to construct the problem
      int  id(int row, int col) const { return indexPosition_ + this->getColBndInd(col)*rows_ + this->getRowBndInd(row); }
      void constraintIndexToCurrentValue(int index) { lBndMat_.col(index) = guessMat_.col(index); uBndMat_.col(index) = guessMat_.col(index); }
      void setAndConstraintIndexToCurrentValue(int index, double value) { guessMat_.col(index).setConstant(value); this->constraintIndexToCurrentValue(index); }
      void setAndConstraintIndexToCurrentValue(int index, OptVector& value) { guessMat_.col(index) = value; this->constraintIndexToCurrentValue(index); }
      void getValues(OptMatrix& lBnd, OptMatrix& uBnd, OptMatrix& guess, int& size, char& type) const { lBnd = lBndMat_; uBnd = uBndMat_; guess = guessMat_; size = this->getNumElements(); type = type_; }

      //! Some getter and setter methods
      int  getNumRows() const { return rows_; }
      int  getNumCols() const { return cols_; }
      int  getStartIndex() const { return indexPosition_; }
      int  getNumElements() const { return rows_*cols_; }
      void getGuessValue(OptMatrix& guess) const { guess.resize(rows_, cols_); guess = guessMat_; }
      void getGuessValueByCol(int index, OptVector& guess) const { guess.resize(rows_,1); guess = guessMat_.col(index); }
      void setGuessValue(const OptMatrix& guess) { guessMat_ = guess.block(0,0,rows_,cols_); guessValueInitialized_ = true; }

    private:
      //! Some helper functions to construct the problem
      int getRowBndInd(int row) const;
      int getColBndInd(int col) const;
      int getBlockLen(int rini, int cini, int rend, int cend) const { return this->id(rend, cend) - this->id(rini, cini) + 1; }

    private:
      char   type_;
      bool   guessValueInitialized_;
      int    indexPosition_, rows_, cols_;
      OptMatrix  lBndMat_, uBndMat_, guessMat_;
  };
}
