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

#include <solver/interface/OptVar.hpp>

namespace solver {

  // Some alternative initialization functions
  void OptimizationVariable::initialize(const char& type, int rows, int cols, double lBnd, double uBnd, int& startIndexInOptVec )
  {
    type_ = type;  rows_ = rows;  cols_ = cols;
    lBndMat_.resize(rows, cols);  lBndMat_.setConstant(lBnd);
    uBndMat_.resize(rows, cols);  uBndMat_.setConstant(uBnd);
    guessMat_.resize(rows, cols); guessMat_.setZero();
    indexPosition_ = startIndexInOptVec;  startIndexInOptVec += rows*cols;
  }

  void OptimizationVariable::initialize(const char& type, int rows, int cols, OptVector& lBnd, OptVector& uBnd, int& startIndexInOptVec )
  {
	assert(rows == lBnd.rows());
	assert(rows == uBnd.rows());
	type_ = type;  rows_ = rows;  cols_ = cols;
    lBndMat_.resize(rows, cols);  lBndMat_.setZero();  lBndMat_.colwise() = lBnd.block(0,0,rows_,1);
    uBndMat_.resize(rows, cols);  uBndMat_.setZero();  uBndMat_.colwise() = uBnd.block(0,0,rows_,1);
    guessMat_.resize(rows, cols); guessMat_.setZero();
    indexPosition_ = startIndexInOptVec;  startIndexInOptVec += rows*cols;
  }

  void OptimizationVariable::initialize(const char& type, int rows, int cols, OptMatrix& lBnd, OptMatrix& uBnd, int& startIndexInOptVec )
  {
    assert(rows == lBnd.rows());   assert(cols == lBnd.cols());
	assert(rows == uBnd.rows());   assert(cols == uBnd.cols());
	type_ = type;  rows_ = rows;  cols_ = cols;
    lBndMat_.resize(rows, cols);  lBndMat_ = lBnd.block(0,0,rows,cols);
    uBndMat_.resize(rows, cols);  uBndMat_ = uBnd.block(0,0,rows,cols);
    guessMat_.resize(rows, cols); guessMat_.setZero();
    indexPosition_ = startIndexInOptVec;  startIndexInOptVec += rows*cols;
  }

  void OptimizationVariable::initialize(const char& type, int rows, int cols, double lBnd, double uBnd, int& startIndexInOptVec, double guess)
  {
    guessValueInitialized_ = true;
    type_ = type;  rows_ = rows;  cols_ = cols;
    lBndMat_.resize(rows, cols);  lBndMat_.setConstant(lBnd);
    uBndMat_.resize(rows, cols);  uBndMat_.setConstant(uBnd);
    guessMat_.resize(rows, cols); guessMat_.setConstant(guess);
    indexPosition_ = startIndexInOptVec;  startIndexInOptVec += rows*cols;
  }

  void OptimizationVariable::initialize(const char& type, int rows, int cols, double lBnd, double uBnd, int& startIndexInOptVec, OptMatrix& guess )
  {
    assert(rows == guess.rows());   assert(cols == guess.cols());
    guessValueInitialized_ = true;
    type_ = type;  rows_ = rows;  cols_ = cols;
    lBndMat_.resize(rows, cols);  lBndMat_.setConstant(lBnd);
    uBndMat_.resize(rows, cols);  uBndMat_.setConstant(uBnd);
    guessMat_.resize(rows, cols); guessMat_ = guess.block(0, 0, rows, cols);
    indexPosition_ = startIndexInOptVec;  startIndexInOptVec += rows*cols;
  }

  void OptimizationVariable::initialize(const char& type, int rows, int cols, OptVector& lBnd, OptVector& uBnd, int& startIndexInOptVec, OptMatrix& guess )
  {
    assert(rows == lBnd.rows());
	assert(rows == uBnd.rows());
	assert(rows == guess.rows());   assert(cols == guess.cols());
	guessValueInitialized_ = true;
    type_ = type;  rows_ = rows;  cols_ = cols;
    lBndMat_.resize(rows, cols);  lBndMat_.setZero(); lBndMat_.colwise() = lBnd.block(0,0,rows_,1);
    uBndMat_.resize(rows, cols);  uBndMat_.setZero(); uBndMat_.colwise() = uBnd.block(0,0,rows_,1);
    guessMat_.resize(rows, cols); guessMat_ = guess.block(0, 0, rows, cols);;
    indexPosition_ = startIndexInOptVec;  startIndexInOptVec += rows*cols;
  }

  void OptimizationVariable::initialize(const char& type, int rows, int cols, OptMatrix& lBnd, OptMatrix& uBnd, int& startIndexInOptVec, OptMatrix& guess )
  {
    assert(rows == lBnd.rows());    assert(cols == lBnd.cols());
	assert(rows == uBnd.rows());    assert(cols == uBnd.cols());
	assert(rows == guess.rows());   assert(cols == guess.cols());
    guessValueInitialized_ = true;
    type_ = type;  rows_ = rows;  cols_ = cols;
    lBndMat_.resize(rows, cols);  lBndMat_ = lBnd.block(0,0,rows,cols);;
    uBndMat_.resize(rows, cols);  uBndMat_ = uBnd.block(0,0,rows,cols);;
    guessMat_.resize(rows, cols); guessMat_ = guess.block(0,0,rows,cols);;
    indexPosition_ = startIndexInOptVec;  startIndexInOptVec += rows*cols;
  }

  int OptimizationVariable::getRowBndInd(int row) const
  {
    if (row >= rows_)  { return rows_-1; }
    else if (row <= 0) { return 0; }
    else               { return row; }
  }

  int OptimizationVariable::getColBndInd(int col) const
  {
    if (col >= cols_)  { return cols_-1; }
    else if (col <= 0) { return 0; }
    else               { return col; }
  }
}
