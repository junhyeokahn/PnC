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

#include <iomanip>
#include <iostream>
#include <solver/optimizer/InfoPrinter.hpp>

namespace solver {

  // OptimizatinInfo class
  OptimizationInfo& OptimizationInfo::operator=(const OptimizationInfo& other)
  {
    if (this!=&other){
      this->get(SolverDoubleParam_Tau) = other.get(SolverDoubleParam_Tau);
      this->get(SolverDoubleParam_Kappa) = other.get(SolverDoubleParam_Kappa);
      this->get(SolverDoubleParam_DualCost) = other.get(SolverDoubleParam_DualCost);
      this->get(SolverDoubleParam_DualityGap) = other.get(SolverDoubleParam_DualityGap);
      this->get(SolverDoubleParam_PrimalCost) = other.get(SolverDoubleParam_PrimalCost);
      this->get(SolverDoubleParam_KappaOverTau) = other.get(SolverDoubleParam_KappaOverTau);
      this->get(SolverDoubleParam_DualResidual) = other.get(SolverDoubleParam_DualResidual);
      this->get(SolverDoubleParam_MeritFunction) = other.get(SolverDoubleParam_MeritFunction);
      this->get(SolverDoubleParam_PrimalResidual) = other.get(SolverDoubleParam_PrimalResidual);
      this->get(SolverDoubleParam_DualInfeasibility) = other.get(SolverDoubleParam_DualInfeasibility);
      this->get(SolverDoubleParam_RelativeDualityGap) = other.get(SolverDoubleParam_RelativeDualityGap);
      this->get(SolverDoubleParam_PrimalInfeasibility) = other.get(SolverDoubleParam_PrimalInfeasibility);
    }
    return *this;
  }

  bool OptimizationInfo::isBetterThan(const OptimizationInfo& info) const
  {
	bool better = false;
    if (this->get(SolverDoubleParam_PrimalInfeasibility)!=SolverSetting::nan && this->get(SolverDoubleParam_KappaOverTau)>1.0) {
      if( info.get(SolverDoubleParam_PrimalInfeasibility)!=SolverSetting::nan ) {
        if ( (this->get(SolverDoubleParam_DualityGap)>0.0 && info.get(SolverDoubleParam_DualityGap)>0.0 && this->get(SolverDoubleParam_DualityGap)<info.get(SolverDoubleParam_DualityGap) ) &&
             (this->get(SolverDoubleParam_PrimalInfeasibility)>0.0 && this->get(SolverDoubleParam_PrimalInfeasibility)<info.get(SolverDoubleParam_PrimalResidual) ) &&
             (this->get(SolverDoubleParam_MeritFunction)>0.0 && this->get(SolverDoubleParam_MeritFunction)<info.get(SolverDoubleParam_MeritFunction) ) ) { better = true; }
      } else {
        if ((this->get(SolverDoubleParam_DualityGap)>0.0 && info.get(SolverDoubleParam_DualityGap)>0.0 && this->get(SolverDoubleParam_DualityGap)<info.get(SolverDoubleParam_DualityGap)) &&
            (this->get(SolverDoubleParam_MeritFunction)>0.0 && this->get(SolverDoubleParam_MeritFunction)<info.get(SolverDoubleParam_MeritFunction))) { better = true; }
      }
    } else {
      if ( (this->get(SolverDoubleParam_DualityGap)>0.0 && info.get(SolverDoubleParam_DualityGap)>0.0 && this->get(SolverDoubleParam_DualityGap)<info.get(SolverDoubleParam_DualityGap)) &&
           (this->get(SolverDoubleParam_PrimalResidual)>0.0 && this->get(SolverDoubleParam_PrimalResidual)<info.get(SolverDoubleParam_PrimalResidual)) &&
           (this->get(SolverDoubleParam_DualResidual)>0.0 && this->get(SolverDoubleParam_DualResidual)<info.get(SolverDoubleParam_DualResidual)) &&
           (this->get(SolverDoubleParam_KappaOverTau)>0.0 && this->get(SolverDoubleParam_KappaOverTau)<info.get(SolverDoubleParam_KappaOverTau)) &&
           (this->get(SolverDoubleParam_MeritFunction)>0.0 && this->get(SolverDoubleParam_MeritFunction)<info.get(SolverDoubleParam_MeritFunction)) ) { better = true; }
    }
    return better;
  }

  // getter and setter methods for double parameters
  double& OptimizationInfo::get(const SolverDoubleParam& param)
  {
    switch (param)
    {
      case SolverDoubleParam_Tau : { return tau_; }
      case SolverDoubleParam_Kappa : { return kappa_; }
      case SolverDoubleParam_DualCost : { return dual_cost_; }
      case SolverDoubleParam_DualityGap : { return duality_gap_; }
      case SolverDoubleParam_StepLength : { return step_length_; }
      case SolverDoubleParam_PrimalCost : { return primal_cost_; }
      case SolverDoubleParam_DualResidual : { return dual_residual_; }
      case SolverDoubleParam_KappaOverTau : { return kappa_over_tau_; }
      case SolverDoubleParam_MeritFunction : { return merit_function_; }
      case SolverDoubleParam_PrimalResidual : { return primal_residual_; }
      case SolverDoubleParam_AffineStepLength : { return affine_step_length; }
      case SolverDoubleParam_DualInfeasibility : { return dual_infeasibility_; }
      case SolverDoubleParam_RelativeDualityGap : { return relative_duality_gap_; }
      case SolverDoubleParam_PrimalInfeasibility : { return primal_infeasibility_; }
      case SolverDoubleParam_CorrectionStepLength : { return correction_step_length_; }
      default: { throw std::runtime_error("OptimizationInfo::get SolverDoubleParam invalid"); break; }
    }
  }

  const double& OptimizationInfo::get(const SolverDoubleParam& param) const
  {
    switch (param)
    {
      case SolverDoubleParam_Tau : { return tau_; }
      case SolverDoubleParam_Kappa : { return kappa_; }
      case SolverDoubleParam_DualCost : { return dual_cost_; }
      case SolverDoubleParam_DualityGap : { return duality_gap_; }
      case SolverDoubleParam_StepLength : { return step_length_; }
      case SolverDoubleParam_PrimalCost : { return primal_cost_; }
      case SolverDoubleParam_DualResidual : { return dual_residual_; }
      case SolverDoubleParam_KappaOverTau : { return kappa_over_tau_; }
      case SolverDoubleParam_MeritFunction : { return merit_function_; }
      case SolverDoubleParam_PrimalResidual : { return primal_residual_; }
      case SolverDoubleParam_AffineStepLength : { return affine_step_length; }
      case SolverDoubleParam_DualInfeasibility : { return dual_infeasibility_; }
      case SolverDoubleParam_RelativeDualityGap : { return relative_duality_gap_; }
      case SolverDoubleParam_PrimalInfeasibility : { return primal_infeasibility_; }
      case SolverDoubleParam_CorrectionStepLength : { return correction_step_length_; }
      default: { throw std::runtime_error("OptimizationInfo::get SolverDoubleParam invalid"); break; }
    }
  }

  // getter and setter methods for integer parameters
  int& OptimizationInfo::get(const SolverIntParam& param)
  {
    switch (param)
    {
      case SolverIntParam_NumIter : { return iteration_; }
      case SolverIntParam_NumRefsLinSolve : { return linear_solve_refinements_; }
      case SolverIntParam_NumRefsLinSolveAffine : { return affine_linear_solve_refinements_; }
      case SolverIntParam_NumRefsLinSolveCorrector : { return correction_linear_solve_refinements_; }
      default: { throw std::runtime_error("OptimizationInfo::get IntParam invalid"); break; }
    }
  }

  const int& OptimizationInfo::get(const SolverIntParam& param) const
  {
    switch (param)
    {
      case SolverIntParam_NumIter : { return iteration_; }
      case SolverIntParam_NumRefsLinSolve : { return linear_solve_refinements_; }
      case SolverIntParam_NumRefsLinSolveAffine : { return affine_linear_solve_refinements_; }
      case SolverIntParam_NumRefsLinSolveCorrector : { return correction_linear_solve_refinements_; }
      default: { throw std::runtime_error("OptimizationInfo::get IntParam invalid"); break; }
    }
  }

  // InfoPrinter class
  void InfoPrinter::display(const Msg& msg, const OptimizationInfo& info)
  {
	if (stgs_->get(SolverBoolParam_Verbose)) {
      switch (msg)
      {
        case Msg::MatrixFactorization:
        {
          std::cout << "Matrix factorization problem" << std::endl;
          break;
        }
        case Msg::SearchDirection:
        {
          std::cout << "Unreliable search direction, recovering best iterate " << info.get(SolverIntParam_NumIter) << std::endl << std::endl;
          break;
        }
        case Msg::NumericalProblem:
        {
          std::cout << "Numerical Problems " << std::setprecision(3)
                    << " (Feasibility = " << std::max(info.get(SolverDoubleParam_DualResidual), info.get(SolverDoubleParam_PrimalResidual))
                    << ", RelGap = " << info.get(SolverDoubleParam_RelativeDualityGap)
                    << ", AbsGap = " << info.get(SolverDoubleParam_DualityGap) << ")" << std::endl << std::endl;
          break;
        }
        case Msg::LineSearchStagnation:
        {
          std::cout << "Line search stagnation, recovering best iterate " << info.get(SolverIntParam_NumIter) << std::endl << std::endl;
          break;
        }
        case Msg::VariablesLeavingCone:
        {
          std::cout << "Variables outside cone, recovering best iterate " << info.get(SolverIntParam_NumIter) << std::endl << std::endl;
          break;
        }
        case Msg::MaxItersReached:
        {
          std::cout << "Max number of iterations reached " << std::setprecision(3)
                    << " (Feasibility = " << std::max(info.get(SolverDoubleParam_DualResidual), info.get(SolverDoubleParam_PrimalResidual))
                    << ", RelGap = " << info.get(SolverDoubleParam_RelativeDualityGap)
                    << ", AbsGap = " << info.get(SolverDoubleParam_DualityGap) << ")" << std::endl << std::endl;
          break;
        }
        case Msg::OptimalityReached:
        {
          std::cout << std::endl << (info.mode()==PrecisionConvergence::Full ? "OPTIMAL" : "Close to OPTIMAL") << std::scientific << std::setprecision(3)
                    << " (Feasibility = " << std::max(info.get(SolverDoubleParam_DualResidual), info.get(SolverDoubleParam_PrimalResidual))
                    << ", RelGap = " << info.get(SolverDoubleParam_RelativeDualityGap)
                    << ", AbsGap = " << info.get(SolverDoubleParam_DualityGap) << ")" << std::endl << std::endl;
          break;
        }
        case Msg::PrimalInfeasibility:
        {
          std::cout << std::endl << (info.mode()==PrecisionConvergence::Full ? "PRIMAL INFEASIBLE" : "Close to PRIMAL INFEASIBLE") << std::scientific << std::setprecision(3)
                    << " (Feasibility = " << info.get(SolverDoubleParam_PrimalInfeasibility) << ")" << std::endl << std::endl;
          break;
        }
        case Msg::DualInfeasibility:
        {
          std::cout << std::endl << (info.mode()==PrecisionConvergence::Full ? "DUAL INFEASIBLE" : "Close to DUAL INFEASIBLE") << std::scientific << std::setprecision(3)
                    << " (Feasibility = " << info.get(SolverDoubleParam_DualInfeasibility) << ")" << std::endl << std::endl;
          break;
        }
        case Msg::OptimizationProgress:
        {
          if (info.get(SolverIntParam_NumIter)==0) {
        	    std::cout << std::endl
        			      << " =============================== OPTIMIZATION PROGRESS =============================== " << std::endl << std::endl;
        	    std::cout << "It        pcost       dcost      gap   pres   dres    k/t    mu     step   sigma     IR" << std::endl;
                std::cout << std::scientific << std::setw(4)  << info.get(SolverIntParam_NumIter)
                                             << std::setw(12) << std::setprecision(3) << info.get(SolverDoubleParam_PrimalCost)
                                             << std::setw(12) << std::setprecision(3) << info.get(SolverDoubleParam_DualCost)
                                             << std::setw(8)  << std::setprecision(0) << info.get(SolverDoubleParam_DualityGap)
                                             << std::setw(7)  << std::setprecision(0) << info.get(SolverDoubleParam_PrimalResidual)
                                             << std::setw(7)  << std::setprecision(0) << info.get(SolverDoubleParam_DualResidual)
                                             << std::setw(7)  << std::setprecision(0) << info.get(SolverDoubleParam_KappaOverTau)
                                             << std::setw(7)  << std::setprecision(0) << info.get(SolverDoubleParam_StepLength)
                          << std::fixed      << std::setw(8)  << std::setprecision(4) << "--- "
                          << std::scientific << std::setw(7)  << std::setprecision(0) << "--- "
                          << std::fixed      << std::setw(4)  << std::setprecision(0) << info.get(SolverIntParam_NumRefsLinSolve)
                                             << std::setw(3)  << std::setprecision(0) << info.get(SolverIntParam_NumRefsLinSolveAffine)
                                             << std::setw(3)  << std::setprecision(0) << "-" << std::endl;
          } else {
        	    std::cout << std::scientific << std::setw(4)  << info.get(SolverIntParam_NumIter)
                                         << std::setw(12) << std::setprecision(3) << info.get(SolverDoubleParam_PrimalCost)
                                         << std::setw(12) << std::setprecision(3) << info.get(SolverDoubleParam_DualCost)
                                         << std::setw(8)  << std::setprecision(0) << info.get(SolverDoubleParam_DualityGap)
                                         << std::setw(7)  << std::setprecision(0) << info.get(SolverDoubleParam_PrimalResidual)
                                         << std::setw(7)  << std::setprecision(0) << info.get(SolverDoubleParam_DualResidual)
                                         << std::setw(7)  << std::setprecision(0) << info.get(SolverDoubleParam_KappaOverTau)
                                         << std::setw(7)  << std::setprecision(0) << info.get(SolverDoubleParam_MeritFunction)
                      << std::fixed      << std::setw(8)  << std::setprecision(4) << info.get(SolverDoubleParam_StepLength)
                      << std::scientific << std::setw(7)  << std::setprecision(0) << info.get(SolverDoubleParam_CorrectionStepLength)
                      << std::fixed      << std::setw(4)  << std::setprecision(0) << info.get(SolverIntParam_NumRefsLinSolve)
                                         << std::setw(3)  << std::setprecision(0) << info.get(SolverIntParam_NumRefsLinSolveAffine)
                                         << std::setw(3)  << std::setprecision(0) << info.get(SolverIntParam_NumRefsLinSolveCorrector) << std::endl;
          }
          break;
        }

      }
	}
  }

}
