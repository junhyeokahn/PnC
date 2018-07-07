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

#include <memory>
#include <solver/interface/SolverParams.hpp>

namespace solver {

  enum class VarType {Continuous};

  struct VarStorage
  {
    int col_no_;
    VarType type_;
    double lb_, ub_, value_, guess_;
  };

  /**
   * Helper class to define an optimization variable, used in the
   * construction of linear and quadratic expressions.
   */
  class Var
  {
    public:
      Var() : var_storage_(nullptr) {};

      int get(SolverIntParam param) const;
      double get(SolverDoubleParam param) const;
      void set(SolverIntParam param, int value);
      void set(SolverDoubleParam param, double value);

      friend class Model;
      friend class LinExpr;

    private:
      Var(int col_no, const VarType& type, double lb, double ub, double guess=0.0);

    private:
	  std::shared_ptr<VarStorage> var_storage_;
  };

}
