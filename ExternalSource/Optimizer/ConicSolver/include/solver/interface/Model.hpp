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

#include <solver/interface/Exprs.hpp>
#include <solver/optimizer/IPSolver.hpp>

namespace solver {

  /**
   * Main class to construct a second-order cone optimization problem.
   * It provides functionality for defining an objective function, linear
   * equality and inequality constraints, second-order cone constraints
   * and heuristics for solving problems with non-convex quadratic constraints.
   */
  class Model
  {
    public:
      Model(){}
      ~Model(){}

      void clean();
      SolverSetting& getStgs() { return stgs_; }
      const SolverSetting& getStgs() const { return stgs_; }
      Var addVar(const VarType& type, double lb, double ub, double guess=0.0);
      void addLinConstr(const LinExpr& lhs, const std::string sense, const LinExpr& rhs);
      void addSocConstr(const DCPQuadExpr& qexpr, const std::string sense, const LinExpr& lexpr);
      void addQuaConstr(const DCPQuadExpr& qexpr, const std::string sense, const LinExpr& expr, const QuadConstrApprox& qapprox = QuadConstrApprox::None );
      void configSetting(const std::string cfg_file, const std::string stgs_vars_yaml = "solver_variables");
      void setObjective(const DCPQuadExpr& qexpr, const LinExpr& expr);
      ExitCode optimize();

    private:
      Cone& getCone() { return cone_; }
      SolverStorage& getStorage() { return stg_; }
      const Cone& getCone() const { return cone_; }
      const SolverStorage& getStorage() const { return stg_; }
      ExitCode solve_problem();
      void build_problem(int iter_id, bool warm_start = false);

    private:
      Cone cone_;
      SolverStorage stg_;
      SolverSetting stgs_;
      InteriorPointSolver ip_solver_;

      DCPQuadExpr objective_;
      std::vector<std::shared_ptr<Var> > vars_;
      int numTrustRegions_, numSoftConstraints_;
      std::vector<LinExpr> leqcons_, lineqcons_;
      std::vector<DCPQuadExpr> qineqcons_, soccons_;

  };
}
