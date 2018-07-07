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

#include <vector>
#include <solver/interface/Var.hpp>

namespace solver {

  /**
   * Helper class to ease the construction of a linear expression (e.g. a*x = b)
   */
  class LinExpr
  {
    public:
      LinExpr(double constant=0.0) { constant_ = constant; vars_.clear(); coeffs_.clear(); }
      LinExpr(Var var, double coeff=1.0) { constant_ = 0.0; vars_.push_back(var); coeffs_.push_back(coeff); }

      double getValue() const;
      static bool isClean(const LinExpr& rhs);
      static LinExpr clean(const LinExpr& rhs);
      int size() const { return vars_.size(); }
      double& getConstant() { return constant_; }
      Var getVar(int i) const { return vars_[i]; }
      double getCoeff(int i) const { return coeffs_[i]; }
      const double& getConstant() const { return constant_; }
      void clear() { constant_ = 0.0; coeffs_.clear(); vars_.clear(); }

      LinExpr operator*(double factor) const;
      LinExpr& operator+=(const LinExpr& rhs);
      LinExpr operator+(const LinExpr& rhs) const { LinExpr result = *this; result += rhs; return result; }
      LinExpr operator-(const LinExpr& rhs) const { LinExpr result = *this; result += rhs*(-1.0); return result; }
      LinExpr operator=(const LinExpr& rhs) { constant_ = rhs.constant_; vars_ = rhs.vars_; coeffs_ = rhs.coeffs_; return *this; }

    private:
      double constant_;
      std::vector<Var> vars_;
      std::vector<double> coeffs_;
  };

  /**
   * Helper class to ease the construction of a quadratic expression
   * (e.g. sum_i (a_i*x + b_i)^2 + (c*x + d))
   */
  class DCPQuadExpr
  {
    public:
      DCPQuadExpr() : lexpr_(0.0), trust_region_(false), soft_constraint_(false) {}
	  ~DCPQuadExpr(){}

	  double getValue() const;
	  void addLinTerm(const LinExpr& lexpr) { lexpr_ += lexpr; }
	  void clear() { lexpr_ = 0.0; qexpr_.clear(); coeffs_.clear(); extra_vars_.clear(); }
	  void addQuaTerm(double coeff, const LinExpr& lexpr) { if (coeff != 0.0) { qexpr_.push_back(lexpr); coeffs_.push_back(coeff); } }

	  LinExpr& lexpr() { return lexpr_; }
	  std::string& sense() { return sense_; }
	  bool& trustRegion() { return trust_region_; }
	  std::vector<LinExpr>& qexpr() { return qexpr_; }
	  std::vector<double>& coeffs() { return coeffs_; }
	  bool& softConstraint() { return soft_constraint_; }
	  std::vector<Var>& extraVars() { return extra_vars_; }

	  const LinExpr& lexpr() const { return lexpr_; }
	  const std::string& sense() const { return sense_; }
	  const bool& trustRegion() const { return trust_region_; }
	  const std::vector<LinExpr>& qexpr() const { return qexpr_; }
	  const std::vector<double>& coeffs() const { return coeffs_; }
	  const bool& softConstraint() const { return soft_constraint_; }
	  const std::vector<Var>& extraVars() const { return extra_vars_; }
	  const Var getVar(int qid, int lid) const { return qexpr_[qid].getVar(lid); }

    private:
      LinExpr lexpr_;
      std::string sense_;
      std::vector<double> coeffs_;
      std::vector<LinExpr> qexpr_;
      std::vector<Var> extra_vars_;
      bool trust_region_, soft_constraint_;
  };
}
