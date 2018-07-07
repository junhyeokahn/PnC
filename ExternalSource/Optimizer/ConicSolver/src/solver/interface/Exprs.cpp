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

#include <cmath>
#include <iostream>
#include <solver/interface/Exprs.hpp>
#include <solver/interface/SolverParams.hpp>

namespace solver {

  // Class for Linear Expressions
  double LinExpr::getValue() const {
    double value = constant_;
    for (unsigned int i = 0; i < vars_.size(); i++)
      value += coeffs_[i]*vars_[i].get(SolverDoubleParam_X);
    return value;
  }

  bool LinExpr::isClean(const LinExpr& rhs) {
    for (int i=1; i<rhs.size(); i++){
	  for (int j=0; j<i; j++) {
		if (rhs.vars_[i].var_storage_->col_no_ == rhs.vars_[j].var_storage_->col_no_)
		  return false;
	  }
	  if (rhs.coeffs_[i] == 0.0)
	    return false;
    }
    return true;
  }

  LinExpr LinExpr::clean(const LinExpr& rhs) {
    LinExpr result;
    result.constant_ = rhs.constant_;

    for (int i=0; i<rhs.size(); i++){
	  bool set = false;
	  for (int j=0; j<result.size(); j++) {
		if (rhs.vars_[i].var_storage_->col_no_ == result.vars_[j].var_storage_->col_no_) {
		  result.coeffs_[j] += (rhs.coeffs_[i]);
		  set = true;
	    }
	  }
	  if (!set && rhs.coeffs_[i] != 0.0) {
        result.coeffs_.push_back(rhs.coeffs_[i]);
        result.vars_.push_back(rhs.vars_[i]);
	  }
    }
    return result;
  }

  LinExpr LinExpr::operator*(double factor) const {
	if (factor != 0.0) {
	  LinExpr result = *this;
	  result.getConstant() *= factor;
	  for (int i=0; i<(int)result.size(); i++)
        result.coeffs_[i] *= factor;
	  return LinExpr::clean(result);
	}
	return LinExpr();
  }

  LinExpr& LinExpr::operator+=(const LinExpr& rhs) {
    this->getConstant() += rhs.getConstant();
    for (int i=0; i<(int)rhs.size(); i++) {
    	  this->vars_.push_back(rhs.vars_[i]);
      this->coeffs_.push_back(rhs.coeffs_[i]);
    }
    *this = LinExpr::clean(*this);
	return *this;
  }

  // Class for Disciplined Convex Quadratic Expressions
  double DCPQuadExpr::getValue() const {
    double value = 0.0;
    for (unsigned int i=0; i<coeffs_.size(); i++)
      value += coeffs_[i]*std::pow(qexpr_[i].getValue(), 2.0);
    return value;
  }

}
