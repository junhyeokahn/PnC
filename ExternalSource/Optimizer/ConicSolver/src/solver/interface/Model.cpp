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

#include <iomanip>
#include <iostream>
#include <solver/interface/Model.hpp>

namespace solver {

  void Model::configSetting(const std::string cfg_file, const std::string stgs_vars_yaml)
  {
	this->getStgs().initialize(cfg_file, stgs_vars_yaml);
  }

  void Model::clean()
  {
	vars_.clear();
	leqcons_.clear();
	soccons_.clear();
	lineqcons_.clear();
	qineqcons_.clear();
	objective_.clear();
	numTrustRegions_ = 0;
	numSoftConstraints_ = 0;
  }

  Var Model::addVar(const VarType& type, double lb, double ub, double guess)
  {
    if (lb > ub) { throw std::runtime_error("Invalid assertion LB <= UB"); }
    if (guess < lb) { guess = lb; if (this->getStgs().get(SolverBoolParam_Verbose)) { std::cerr << "Warning: Guess < LB, Setting: Guess = LB" << std::endl; } }
    else  if (guess > ub) { guess = ub; if (this->getStgs().get(SolverBoolParam_Verbose)) { std::cerr << "Warning: Guess > UB, Setting: Guess = UB" << std::endl; } }

    Var var = Var(vars_.size(), type, lb, ub, guess);
    vars_.push_back( std::make_shared<Var>( var ) );
    return var;
  }

  // Linear Constraint: left_hand_side [< = >] right_hand_side
  void Model::addLinConstr(const LinExpr& lhs, const std::string sense, const LinExpr& rhs)
  {
    if (sense == "=") { leqcons_.push_back(lhs-rhs); }
    else if (sense == "<") { lineqcons_.push_back(lhs-rhs); }
    else if (sense == ">") { lineqcons_.push_back(rhs-lhs); }
    else { throw std::runtime_error("Invalid sense on Linear Constraint"); }
  }

  // Disciplined Convex Quadratic Constraint: Sum coeffs[i]* (DCP.qexpr[i])^2 + (DCP.lexpr - lexpr) [< =] 0.0
  void Model::addQuaConstr(const DCPQuadExpr& qexpr, const std::string sense, const LinExpr& lexpr, const QuadConstrApprox& qapprox)
  {
    DCPQuadExpr qstg = qexpr;
    qstg.lexpr() = qexpr.lexpr() - lexpr;
    switch (qapprox) {
      case QuadConstrApprox::TrustRegion: { numTrustRegions_ += 1;  qstg.trustRegion() = true; break; }
      case QuadConstrApprox::SoftConstraint: { numSoftConstraints_ +=1;  qstg.softConstraint() = true; break; }
      default: { break; }
    }
    for (unsigned int i=0; i<qstg.coeffs().size(); i++)
	  qstg.extraVars().push_back(this->addVar(VarType::Continuous, 0.0, 1.0, 0.5));
    if (qapprox == QuadConstrApprox::SoftConstraint) { qstg.extraVars().push_back(this->addVar(VarType::Continuous, 0.0, 1.0, 0.5)); }

    if (sense == "<") { qineqcons_.push_back(qstg); }
    else { throw std::runtime_error("Invalid sense on Quadratic Constraint"); }
  }

  // Second-Order Cone Constraint: SQRT( Sum (DCP.qexpr[i])^2 ) + (DCP.lexpr - lexpr) [< =] 0.0
  void Model::addSocConstr(const DCPQuadExpr& qexpr, const std::string sense, const LinExpr& lexpr)
  {
    DCPQuadExpr qstg = qexpr;
    qstg.trustRegion() = false;
    qstg.lexpr() = qexpr.lexpr() - lexpr;
    qstg.extraVars().push_back(this->addVar(VarType::Continuous, 0.0, 1.0, 0.5));

    if (sense == "<") { soccons_.push_back(qstg); }
    else { throw std::runtime_error("Invalid sense on Second-Order Cone Constraint"); }
  }


  // Quadratic Objective: min Sum DCP.coeffs[i]*(DCP.qexpr[i])^2 + (DCP.lexpr+lexpr)
  void Model::setObjective(const DCPQuadExpr& qexpr, const LinExpr& lexpr)
  {
    DCPQuadExpr qstg;
    objective_.qexpr() = qexpr.qexpr();
    objective_.coeffs() = qexpr.coeffs();
    objective_.lexpr() = qexpr.lexpr() + lexpr;
    for (int i=0; i<(int)qexpr.coeffs().size(); i++)
      objective_.extraVars().push_back(this->addVar(VarType::Continuous, 0.0, 1.0, 0.5));
  }

  // Translate problem to standard conic form
  void Model::build_problem(int iter_id, bool warm_start)
  {
    // problem size
	numTrustRegions_ = 0;
	numSoftConstraints_ = 0;
	int row_start = 0, row_offset = 0;
	int mextra = objective_.extraVars().size() + soccons_.size();
    for (int id=0; id<(int)qineqcons_.size(); id++) {
	  mextra += qineqcons_[id].coeffs().size();
	  if (qineqcons_[id].trustRegion()) { numTrustRegions_ += 1; }
	  if (qineqcons_[id].softConstraint()) { numSoftConstraints_ += 1; }
	  if (qineqcons_[id].softConstraint() && !warm_start) { mextra += 1; }
    }

	int nvars  = vars_.size();
	int nleq   = leqcons_.size();
	int nlineq = lineqcons_.size() + qineqcons_.size() +
			     soccons_.size() + (warm_start == true ? 0 : numTrustRegions_);
	Eigen::VectorXi q(mextra); q.setConstant(3);
    for (int id=0; id<(int)soccons_.size(); id++)
      q.tail(soccons_.size())[id] = soccons_[id].coeffs().size()+1;

	this->getCone().initialize(nvars, nleq, nlineq, q);
	this->getStorage().initialize(cone_, stgs_);
	this->getStorage().cleanCoeffs();

    // Linear equality constraints
    for (int row_id=0; row_id<(int)leqcons_.size(); row_id++) {
      for (int var_id=0; var_id<(int)leqcons_[row_id].size(); var_id++) {
    	    this->getStorage().addCoeff(Eigen::Triplet<double>(row_start+row_id,
			                        leqcons_[row_id].getVar(var_id).get(SolverIntParam_ColNum),
						            leqcons_[row_id].getCoeff(var_id)), true);
	    this->getStorage().b()[row_start+row_id] = -leqcons_[row_id].getConstant();
	  }
    }

    // Linear inequality constraints
    for (int row_id=0; row_id<(int)lineqcons_.size(); row_id++) {
	  for (int var_id=0; var_id<(int)lineqcons_[row_id].size(); var_id++) {
		this->getStorage().addCoeff(Eigen::Triplet<double>(row_start+row_offset+row_id,
			                        lineqcons_[row_id].getVar(var_id).get(SolverIntParam_ColNum),
						            lineqcons_[row_id].getCoeff(var_id)));
	    this->getStorage().h()[row_start+row_id] = -lineqcons_[row_id].getConstant();
	  }
    }
    row_start += lineqcons_.size();

    // Quadratic inequality constraints linear part
    for (int row_id=0; row_id<(int)qineqcons_.size(); row_id++) {
      for (int extra_var_id=0; extra_var_id<(int)qineqcons_[row_id].coeffs().size(); extra_var_id++) {
    	    this->getStorage().addCoeff(Eigen::Triplet<double>(row_start+row_offset+row_id,
    		                            qineqcons_[row_id].extraVars()[extra_var_id].get(SolverIntParam_ColNum),
			                        qineqcons_[row_id].coeffs()[extra_var_id]));
      }
      for (int lvar_id=0; lvar_id<(int)qineqcons_[row_id].lexpr().size(); lvar_id++) {
    	    this->getStorage().addCoeff(Eigen::Triplet<double>(row_start+row_offset+row_id,
    		                            qineqcons_[row_id].lexpr().getVar(lvar_id).get(SolverIntParam_ColNum),
			                        qineqcons_[row_id].lexpr().getCoeff(lvar_id)));
      }
      this->getStorage().h()[row_start+row_id] = -qineqcons_[row_id].lexpr().getConstant();
    }
    row_start += qineqcons_.size();

    // Second order cone constraints linear part
    for (int row_id=0; row_id<(int)soccons_.size(); row_id++) {
         int extra_var_id = 0;
         this->getStorage().addCoeff(Eigen::Triplet<double>(row_start+row_offset+row_id,
                                  soccons_[row_id].extraVars()[extra_var_id].get(SolverIntParam_ColNum),
                                  1.0));
      for (int lvar_id=0; lvar_id<(int)soccons_[row_id].lexpr().size(); lvar_id++) {
           this->getStorage().addCoeff(Eigen::Triplet<double>(row_start+row_offset+row_id,
                                    soccons_[row_id].lexpr().getVar(lvar_id).get(SolverIntParam_ColNum),
                                    soccons_[row_id].lexpr().getCoeff(lvar_id)));
      }
      this->getStorage().h()[row_start+row_id] = -soccons_[row_id].lexpr().getConstant();
    }
    row_start += soccons_.size();

    // Objective linear part
    for (int var_id=0; var_id<(int)objective_.lexpr().size(); var_id++)
      this->getStorage().c()[objective_.lexpr().getVar(var_id).get(SolverIntParam_ColNum)] = objective_.lexpr().getCoeff(var_id);

    if (!warm_start) {
      // Quadratic constraints with trust region
      int row_count = 0;
      for (int row_id=0; row_id<(int)qineqcons_.size(); row_id++) {
    	    if (qineqcons_[row_id].trustRegion()) {
    	      this->getStorage().h()[row_start+row_count] = 0.0;
          for (int qvar_id=0; qvar_id<(int)qineqcons_[row_id].qexpr().size(); qvar_id++) {
            for (int lvar_id=0; lvar_id<(int)qineqcons_[row_id].qexpr()[qvar_id].size(); lvar_id++) {
              this->getStorage().addCoeff(Eigen::Triplet<double>(row_start+row_offset+row_count,
          		                          qineqcons_[row_id].getVar(qvar_id, lvar_id).get(SolverIntParam_ColNum),
  						                 -2.0*qineqcons_[row_id].coeffs()[qvar_id]*qineqcons_[row_id].qexpr()[qvar_id].getValue()*qineqcons_[row_id].qexpr()[qvar_id].getCoeff(lvar_id)));
              this->getStorage().h()[row_start+row_count] -= 2.0*qineqcons_[row_id].coeffs()[qvar_id]*qineqcons_[row_id].qexpr()[qvar_id].getValue()*qineqcons_[row_id].qexpr()[qvar_id].getCoeff(lvar_id)*qineqcons_[row_id].getVar(qvar_id, lvar_id).get(SolverDoubleParam_X);
            }
          }

          for (int lvar_id=0; lvar_id<(int)qineqcons_[row_id].lexpr().size(); lvar_id++) {
        	    this->getStorage().addCoeff(Eigen::Triplet<double>(row_start+row_offset+row_count,
      		                            qineqcons_[row_id].lexpr().getVar(lvar_id).get(SolverIntParam_ColNum),
  						               -qineqcons_[row_id].lexpr().getCoeff(lvar_id)));
          }
          this->getStorage().h()[row_start+row_count] += qineqcons_[row_id].lexpr().getConstant() + qineqcons_[row_id].getValue() + std::pow(this->getStgs().get(SolverDoubleParam_TrustRegionThreshold), iter_id);
          row_count += 1;
    	    }
      }
      row_start += numTrustRegions_;
    }

    // Objective quadratic part
    for (int extra_var_id=0; extra_var_id<(int)objective_.extraVars().size(); extra_var_id++) {
      this->getStorage().addCoeff(Eigen::Triplet<double>(row_start+row_offset+3*extra_var_id+0,
				                  objective_.extraVars()[extra_var_id].get(SolverIntParam_ColNum),
				                 -1.0));
      this->getStorage().addCoeff(Eigen::Triplet<double>(row_start+row_offset+3*extra_var_id+1,
				                  objective_.extraVars()[extra_var_id].get(SolverIntParam_ColNum),
				                 -1.0));
	  for (int qvar_id=0; qvar_id<(int)objective_.qexpr()[extra_var_id].size(); qvar_id++) {
		this->getStorage().addCoeff(Eigen::Triplet<double>(row_start+row_offset+3*extra_var_id+2,
					                objective_.qexpr()[extra_var_id].getVar(qvar_id).get(SolverIntParam_ColNum),
					               -2.0*objective_.qexpr()[extra_var_id].getCoeff(qvar_id)));
	  }
	  this->getStorage().h()[row_start+3*extra_var_id+0] =  1.0;
	  this->getStorage().h()[row_start+3*extra_var_id+1] = -1.0;
	  this->getStorage().h()[row_start+3*extra_var_id+2] =  2.0*objective_.qexpr()[extra_var_id].getConstant();
	  this->getStorage().c()[objective_.extraVars()[extra_var_id].get(SolverIntParam_ColNum)] = objective_.coeffs()[extra_var_id];
    }
    row_start += 3*objective_.extraVars().size();

    if (!warm_start) {
      // Adding quadratic objective terms due to soft constraints
      int row_count = 0;
      for (int row_id=0; row_id<(int)qineqcons_.size(); row_id++) {
    	    if (qineqcons_[row_id].softConstraint()) {
    	      this->getStorage().h()[row_start+3*row_count+0] =  1.0;
    	      this->getStorage().h()[row_start+3*row_count+1] = -1.0;
    	      this->getStorage().h()[row_start+3*row_count+2] =  0.0;
    	      this->getStorage().addCoeff(Eigen::Triplet<double>(row_start+row_offset+3*row_count+0,
    	    		                          qineqcons_[row_id].extraVars().back().get(SolverIntParam_ColNum),
    					                 -1.0));
    	      this->getStorage().addCoeff(Eigen::Triplet<double>(row_start+row_offset+3*row_count+1,
    	    		                          qineqcons_[row_id].extraVars().back().get(SolverIntParam_ColNum),
    					                 -1.0));
          for (int qvar_id=0; qvar_id<(int)qineqcons_[row_id].qexpr().size(); qvar_id++) {
            for (int lvar_id=0; lvar_id<(int)qineqcons_[row_id].qexpr()[qvar_id].size(); lvar_id++) {
              this->getStorage().addCoeff(Eigen::Triplet<double>(row_start+row_offset+3*row_count+2,
          		                          qineqcons_[row_id].getVar(qvar_id, lvar_id).get(SolverIntParam_ColNum),
  						                 -4.0*qineqcons_[row_id].coeffs()[qvar_id]*qineqcons_[row_id].qexpr()[qvar_id].getValue()*qineqcons_[row_id].qexpr()[qvar_id].getCoeff(lvar_id)));
              this->getStorage().h()[row_start+3*row_count+2] -= 2.0*qineqcons_[row_id].coeffs()[qvar_id]*qineqcons_[row_id].qexpr()[qvar_id].getValue()*qineqcons_[row_id].qexpr()[qvar_id].getCoeff(lvar_id)*qineqcons_[row_id].getVar(qvar_id, lvar_id).get(SolverDoubleParam_X);
            }
          }
          for (int lvar_id=0; lvar_id<(int)qineqcons_[row_id].lexpr().size(); lvar_id++) {
        	    this->getStorage().addCoeff(Eigen::Triplet<double>(row_start+row_offset+3*row_count+2,
      		                            qineqcons_[row_id].lexpr().getVar(lvar_id).get(SolverIntParam_ColNum),
  						               -2.0*qineqcons_[row_id].lexpr().getCoeff(lvar_id) ));
          }
          this->getStorage().h()[row_start+3*row_count+2] += qineqcons_[row_id].lexpr().getConstant() + qineqcons_[row_id].getValue();
          this->getStorage().h()[row_start+3*row_count+2] = 2.0*this->getStorage().h()[row_start+3*row_count+2];
          this->getStorage().c()[qineqcons_[row_id].extraVars().back().get(SolverIntParam_ColNum)] = this->getStgs().get(SolverDoubleParam_SoftConstraintWeight);
          row_count += 1;
    	    }
      }
      row_start += 3*numSoftConstraints_;
    }

    // Quadratic inequality constraints quadratic part
    for (int row_id=0; row_id<(int)qineqcons_.size(); row_id++) {
      for (int extra_var_id=0; extra_var_id<(int)qineqcons_[row_id].coeffs().size(); extra_var_id++) {
    	    this->getStorage().addCoeff(Eigen::Triplet<double>(row_start+row_offset+3*extra_var_id+0,
				                    qineqcons_[row_id].extraVars()[extra_var_id].get(SolverIntParam_ColNum),
				                   -1.0));
    	    this->getStorage().addCoeff(Eigen::Triplet<double>(row_start+row_offset+3*extra_var_id+1,
				                    qineqcons_[row_id].extraVars()[extra_var_id].get(SolverIntParam_ColNum),
				                   -1.0));
	    for (int qvar_id=0; qvar_id<(int)qineqcons_[row_id].qexpr()[extra_var_id].size(); qvar_id++) {
	      this->getStorage().addCoeff(Eigen::Triplet<double>(row_start+row_offset+3*extra_var_id+2,
					                  qineqcons_[row_id].qexpr()[extra_var_id].getVar(qvar_id).get(SolverIntParam_ColNum),
					                 -2.0*qineqcons_[row_id].qexpr()[extra_var_id].getCoeff(qvar_id)));
	    }
	    this->getStorage().h()[row_start+3*extra_var_id+0] =  1.0;
	    this->getStorage().h()[row_start+3*extra_var_id+1] = -1.0;
	    this->getStorage().h()[row_start+3*extra_var_id+2] =  2.0*qineqcons_[row_id].qexpr()[extra_var_id].getConstant();
	  }
	  row_start += 3*qineqcons_[row_id].coeffs().size();
    }

    // Second order cone constraints quadratic part
    for (int row_id=0; row_id<(int)soccons_.size(); row_id++) {
      int size = soccons_[row_id].coeffs().size();
      this->getStorage().addCoeff(Eigen::Triplet<double>(row_start+row_offset+0,
                                  soccons_[row_id].extraVars()[0].get(SolverIntParam_ColNum),
                                 -1.0));
      this->getStorage().h()[row_start+0] = 0.0;

      for (int qvar_id=0; qvar_id<(int)soccons_[row_id].qexpr().size(); qvar_id++) {
        for (int lvar_id=0; lvar_id<(int)soccons_[row_id].qexpr()[qvar_id].size(); lvar_id++) {
          this->getStorage().addCoeff(Eigen::Triplet<double>(row_start+row_offset+qvar_id+1,
                                      soccons_[row_id].qexpr()[qvar_id].getVar(lvar_id).get(SolverIntParam_ColNum),
                                     -soccons_[row_id].qexpr()[qvar_id].getCoeff(lvar_id)));
        }
        this->getStorage().h()[row_start+qvar_id+1] = soccons_[row_id].qexpr()[qvar_id].getConstant();
      }
      row_start += size+1;
    }

    this->getStorage().initializeMatrices();
    ip_solver_.initialize(this->getStorage(), this->getCone(), this->getStgs());
  }

  ExitCode Model::solve_problem()
  {
    ExitCode exit_code = ExitCode::Indeterminate;
    exit_code = ip_solver_.optimize();
    for (int i=0; i<(int)vars_.size(); i++)
      vars_[i]->set(SolverDoubleParam_X, ip_solver_.optsol().x()[i]);
    return exit_code;
  }

  ExitCode Model::optimize()
  {
	ExitCode exit_code = ExitCode::Indeterminate;

    // warm start solution
    if (this->getStgs().get(SolverIntParam_WarmStartIters)>0 && (numTrustRegions_>0 || numSoftConstraints_>0)) {
      this->getStgs().set(SolverIntParam_MaxIters, this->getStgs().get(SolverIntParam_WarmStartIters));
      this->build_problem(0, true);
      exit_code =this->solve_problem();
    }

    if (this->getStgs().get(SolverIntParam_SolverMaxIters)>0) {
      // solve problem using convex conic solver
      this->getStgs().set(SolverIntParam_MaxIters, this->getStgs().get(SolverIntParam_SolverMaxIters));
      this->build_problem(1);
      exit_code = this->solve_problem();
    }

    // perform refinements if required
    if (this->getStgs().get(SolverIntParam_NumberRefinementsTrustRegion) > 0 && (numTrustRegions_>0 || numSoftConstraints_>0)) {
      for (int ref=1; ref<=this->getStgs().get(SolverIntParam_NumberRefinementsTrustRegion); ref++) {
      	this->getStgs().set(SolverIntParam_MaxIters, this->getStgs().get(SolverIntParam_SolverMaxIters));
        this->build_problem(ref+1);
        exit_code = this->solve_problem();
      }
    }
    return exit_code;
  }

}
