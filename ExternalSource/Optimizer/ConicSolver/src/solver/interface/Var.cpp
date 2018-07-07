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

#include <iostream>
#include <stdexcept>
#include <solver/interface/Var.hpp>

namespace solver {

  Var::Var(int col_no, const VarType& type, double lb, double ub, double guess)
  {
    var_storage_.reset( new VarStorage() );
    var_storage_->lb_ = lb;
    var_storage_->ub_ = ub;
    var_storage_->type_ = type;
    var_storage_->value_ = guess;
    var_storage_->guess_ = guess;
    var_storage_->col_no_ = col_no;
  }

  int Var::get(SolverIntParam param) const
  {
    int value;
    if (var_storage_ == nullptr)
      throw std::runtime_error("Variable not initialized");

    switch (param) {
      case SolverIntParam_ColNum : { value = this->var_storage_->col_no_; break; }
      default: { throw std::runtime_error("Var::get IntParam"); break; }
    }
    return value;
  }

  void Var::set(SolverIntParam param, int value)
  {
    switch (param) {
      case SolverIntParam_ColNum : { this->var_storage_->col_no_ = value; break; }
      default: { throw std::runtime_error("Var::set IntParam"); break; }
    }
  }

  double Var::get(SolverDoubleParam param) const
  {
    double value;
    if (var_storage_ == nullptr)
      throw std::runtime_error("Variable not initialized");

    switch (param) {
      case SolverDoubleParam_X : { value = this->var_storage_->value_; break; }
      case SolverDoubleParam_LB : { value = this->var_storage_->lb_; break; }
      case SolverDoubleParam_UB : { value = this->var_storage_->ub_; break; }
      case SolverDoubleParam_Guess : { value = this->var_storage_->guess_; break; }
      default: { throw std::runtime_error("Var::get SolverDoubleParam"); break; }
    }
    return value;
  }

  void Var::set(SolverDoubleParam param, double value)
  {
    switch (param) {
      case SolverDoubleParam_X : { this->var_storage_->value_ = value; break; }
      case SolverDoubleParam_LB : { this->var_storage_->lb_ = value; break; }
      case SolverDoubleParam_UB : { this->var_storage_->ub_ = value; break; }
      case SolverDoubleParam_Guess : { this->var_storage_->guess_ = value; break; }
      default: { throw std::runtime_error("Var::set SolverDoubleParam"); break; }
    }
  }

}
