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

#include <solver/optimizer/IPSolver.hpp>

namespace solver {

  inline double dotProduct(int n, double* x, double* y)
  {
    double z = 0; int i;
    for( i=0; i<n; i++ ){ z += x[i]*y[i]; }
    return z;
  }

  void InteriorPointSolver::initialize(SolverStorage& stg, Cone& cone, SolverSetting& stgs)
  {
    // setup problem
	stg_ = &stg;
	stgs_ = &stgs;
	cone_ = &cone;
    this->internalInitialization();
  }

  void InteriorPointSolver::internalInitialization()
  {
	// setup message printer
	this->getPrinter().initialize(this->getStgs());

	// equilibration of problem data
	this->getEqRoutine().setEquilibration(this->getCone(), this->getStgs(), this->getStg());
	this->getStg().At() = this->getStg().A().transpose();
	this->getStg().Gt() = this->getStg().G().transpose();
	this->getLinSolver().initialize(this->getCone(), this->getStgs(), this->getStg());

	// initialize problem variables
  	rho_.initialize(this->getCone());
    opt_.initialize(this->getCone());
    res_.initialize(this->getCone());
    RHS1_.initialize(this->getCone());
    RHS2_.initialize(this->getCone());
    lbar_.initialize(this->getCone());
    dopt1_.initialize(this->getCone());
    dopt2_.initialize(this->getCone());
    sigma_.initialize(this->getCone());
    lambda_.initialize(this->getCone());
    best_opt_.initialize(this->getCone());
    ds_combined_.initialize(this->getCone());
    dz_combined_.initialize(this->getCone());
    ds_affine_by_W_.initialize(this->getCone());
    W_times_dz_affine_.initialize(this->getCone());
  }

  ExitCode InteriorPointSolver::initializeVariables()
  {
	// get scalings of problem data
	inires_x_ = std::max(1.0, this->getStg().c().norm());
	inires_y_ = std::max(1.0, this->getStg().b().norm());
	inires_z_ = std::max(1.0, this->getStg().h().norm());

	// initialize KKT matrix and perform numeric factorization
	this->getLinSolver().initializeMatrix();
	if ( this->getLinSolver().numericFactorization() != FactStatus::Optimal ){
	  this->getPrinter().display(Msg::MatrixFactorization, this->getInfo());
      return ExitCode::Indeterminate;
    }

	// initialize primal variables
	int* invPerm = this->getLinSolver().invPerm().indices().data();
	for (int i=0; i<this->getCone().numVars(); i++) { RHS1_[invPerm[i]] = 0; }
	for (int i=0; i<this->getCone().numLeq(); i++ ) { RHS1_[invPerm[this->getCone().numVars()+i]] = this->getStg().b()[i]; }
	for (int i=0; i<this->getCone().sizeLpc(); i++) { RHS1_[invPerm[this->getCone().lpConeStart()+i]] = this->getStg().h()[i]; }
	for (int l=0; l<this->getCone().numSoc(); l++ ){
	  for (int i=0; i<this->getCone().sizeSoc(l); i++)
        RHS1_[invPerm[this->getCone().extStartSoc(l)+i]] = this->getStg().cbh()[this->getCone().optStartSoc(l)+i];
	}

	this->getInfo().get(SolverIntParam_NumRefsLinSolve) = this->getLinSolver().solve(RHS1_, dopt2_, true);
	opt_.x() = dopt2_.x();
	opt_.s() = -dopt2_.z();
	this->getCone().conicProjection(opt_.s());

	// initialize dual variables
	for (int i=0; i<this->getCone().numVars(); i++){ RHS2_[invPerm[i]] = -this->getStg().c()[i]; }

	this->getInfo().get(SolverIntParam_NumRefsLinSolveAffine) = this->getLinSolver().solve(RHS2_, dopt1_, true);
	opt_.y() = dopt1_.y();
	opt_.z() = dopt1_.z();
	this->getCone().conicProjection(opt_.z());

	// initialize variables for optimization
	for (int i=0; i<this->getCone().numVars(); i++) { RHS1_[invPerm[i]] = -this->getStg().c()[i]; }
	opt_.kappa() = opt_.tau() = 1.0;
	this->getInfo().get(SolverDoubleParam_StepLength) = this->getInfo().get(SolverDoubleParam_AffineStepLength) = 0.0;

	return ExitCode::Optimal;
  }

  void InteriorPointSolver::computeResiduals()
  {
	this->getLinSolver().matrixTransposeTimesVector(this->getStg().A(), opt_.y(), res_.x(), false, true);
	this->getLinSolver().matrixTransposeTimesVector(this->getStg().G(), opt_.z(), res_.x(), false, false);
	residual_x_ = res_.x().norm();

	this->getLinSolver().matrixTransposeTimesVector(this->getStg().At(), opt_.x(), res_.y(), true, true);
    residual_y_ = res_.y().norm();

	this->getLinSolver().matrixTransposeTimesVector(this->getStg().Gt(), opt_.x(), res_.z(), true, true);
	res_.z() += opt_.s();
	residual_z_ = res_.z().norm();

	cx_ = this->getStg().c().dot(opt_.x());
	by_ = this->getStg().b().dot(opt_.y());
	hz_ = this->getStg().h().dot(opt_.z());
	res_ -= opt_.tau()*this->getStg().cbh();
	residual_t_ = opt_.kappa() + cx_ + by_ + hz_;
  }

  double InteriorPointSolver::lineSearch(const Eigen::Ref<const Eigen::VectorXd>& dsvec, const Eigen::Ref<const Eigen::VectorXd>& dzvec,
		                                 double tau, double dtau, double kappa, double dkappa)
  {
	int conestart, conesize;
	const double *lk, *dsk, *dzk;
	double *lkbar, *rhok, *sigmak;
	const double* ds = dsvec.data();
	const double* dz = dzvec.data();
	const double* lambda = lambda_.data();
	double conicres_lk, conicres_ds, conicres_dz, inv_conicres, factor1, factor2;

    // Linear cone
    double rhomin = ds[0]/lambda[0];
    double sigmamin = dz[0]/lambda[0];
    for (int i=1; i<this->getCone().sizeLpc(); i++){
      if (ds[i]/lambda[i]<rhomin) { rhomin = ds[i]/lambda[i]; }
      if (dz[i]/lambda[i]<sigmamin) { sigmamin = dz[i]/lambda[i]; }
    }
    double alpha = std::min(sigmamin,rhomin)<0.0 ? -1.0/std::min(sigmamin,rhomin) : 1.0/this->getStgs().get(SolverDoubleParam_DynamicRegularizationThresh);

    // Tau and kappa
    if (-tau/dtau>0 && -tau/dtau<alpha) { alpha = -tau/dtau; }
    if (-kappa/dkappa>0 && -kappa/dkappa<alpha) { alpha = -kappa/dkappa; }

	// Second order cone
	for (int i=0; i<this->getCone().numSoc(); i++) {
	  // indices
	  conesize = this->getCone().sizeSoc(i);
	  conestart = this->getCone().startSoc(i);
	  dsk = ds + conestart;  dzk = dz + conestart;
	  lk = lambda + conestart;  lkbar = lbar_.data() + conestart;
	  rhok = rho_.data() + conestart;  sigmak = sigma_.data() + conestart;

	  Eigen::Map<const Eigen::VectorXd> eig_lk(lk,conesize);
	  Eigen::Map<const Eigen::VectorXd> eig_rhok(rhok, conesize);
	  Eigen::Map<const Eigen::VectorXd> eig_sigmak(sigmak, conesize);

	  // find residuals
	  conicres_lk = this->getCone().conicResidual(eig_lk);
	  if (conicres_lk <= 0.0) { continue; }

	  for (int j=0; j<conesize; j++) { lkbar[j] = lk[j]/std::sqrt(conicres_lk); }
      conicres_ds = lkbar[0]*dsk[0]; for (int j=1; j<conesize; j++) { conicres_ds -= lkbar[j]*dsk[j]; }
      conicres_dz = lkbar[0]*dzk[0]; for (int j=1; j<conesize; j++) { conicres_dz -= lkbar[j]*dzk[j]; }

      // construct rhok and sigmak
      inv_conicres = 1.0/std::sqrt(conicres_lk);
	  factor1 = (conicres_ds+dsk[0])/(lkbar[0]+1);
	  factor2 = (conicres_dz+dzk[0])/(lkbar[0]+1);

      rhok[0] = inv_conicres * conicres_ds;
      sigmak[0] = inv_conicres * conicres_dz;
	  for (int j=1; j<conesize; j++) { rhok[j] = inv_conicres*(dsk[j]-factor1*lkbar[j]); }
	  for (int j=1; j<conesize; j++) { sigmak[j] = inv_conicres*(dzk[j]-factor2*lkbar[j]); }

	  // update alpha
	  alpha = std::min(alpha,1.0/std::max(std::max(eig_rhok.tail(conesize-1).norm()-rhok[0],eig_sigmak.tail(conesize-1).norm()-sigmak[0]),0.0));
	}
	return std::max(std::min(alpha,this->getStgs().get(SolverDoubleParam_MaximumStepLength)),this->getStgs().get(SolverDoubleParam_MinimumStepLength));
  }

  void InteriorPointSolver::updateStatistics()
  {
	this->getInfo().get(SolverDoubleParam_PrimalCost) = cx_ / opt_.tau();
	this->getInfo().get(SolverDoubleParam_DualityGap) = opt_.s().dot(opt_.z());
	this->getInfo().get(SolverDoubleParam_DualCost) = -(hz_ + by_) / opt_.tau();
	this->getInfo().get(SolverDoubleParam_KappaOverTau) = opt_.kappa() / opt_.tau();
	this->getInfo().get(SolverDoubleParam_MeritFunction) = (this->getInfo().get(SolverDoubleParam_DualityGap) + opt_.kappa()*opt_.tau()) / (this->getCone().sizeCone()+1);

	// relative duality gap
	if      (this->getInfo().get(SolverDoubleParam_PrimalCost) < 0.0) { this->getInfo().get(SolverDoubleParam_RelativeDualityGap) = this->getInfo().get(SolverDoubleParam_DualityGap) / (-this->getInfo().get(SolverDoubleParam_PrimalCost)); }
	else if (this->getInfo().get(SolverDoubleParam_DualCost)   > 0.0) { this->getInfo().get(SolverDoubleParam_RelativeDualityGap) = this->getInfo().get(SolverDoubleParam_DualityGap) /   this->getInfo().get(SolverDoubleParam_DualCost); }
	else                                                              { this->getInfo().get(SolverDoubleParam_RelativeDualityGap) = SolverSetting::nan; }

	// residuals
	this->getInfo().get(SolverDoubleParam_PrimalResidual) = std::max(res_.y().norm()/std::max(inires_y_+opt_.x().norm(),1.0),
                                                                     res_.z().norm()/std::max(inires_z_+opt_.x().norm()+opt_.s().norm(),1.0)) / opt_.tau();
	this->getInfo().get(SolverDoubleParam_DualResidual) = res_.x().norm()/std::max(inires_x_+opt_.y().norm()+opt_.z().norm(),1.0) / opt_.tau();

    // infeasibility measures
	this->getInfo().get(SolverDoubleParam_PrimalInfeasibility) = (hz_ + by_)/std::max(opt_.y().norm()+opt_.z().norm(),1.0) < -this->getStgs().get(SolverDoubleParam_DualityGapRelTol) ? residual_x_ / std::max(opt_.y().norm()+opt_.z().norm(),1.0) : SolverSetting::nan;
	this->getInfo().get(SolverDoubleParam_DualInfeasibility) = cx_/std::max(opt_.x().norm(),1.0) < -this->getStgs().get(SolverDoubleParam_DualityGapRelTol) ? std::max(residual_y_/std::max(opt_.x().norm(),1.0), residual_z_/std::max(opt_.x().norm()+opt_.s().norm(),1.0)) : SolverSetting::nan;
  }

  ExitCode InteriorPointSolver::convergenceCheck(const PrecisionConvergence& mode)
  {
	this->getInfo().mode() = mode;
    double feastol, abstol, reltol;
    ExitCode exitcode = ExitCode::NotConverged;

    // set accuracy
    if ( mode == PrecisionConvergence::Full) {
      feastol = this->getStgs().get(SolverDoubleParam_FeasibilityTol);
      abstol  = this->getStgs().get(SolverDoubleParam_DualityGapAbsTol);
      reltol  = this->getStgs().get(SolverDoubleParam_DualityGapRelTol);
    } else {
      feastol = this->getStgs().get(SolverDoubleParam_FeasibilityTolInacc);
      abstol  = this->getStgs().get(SolverDoubleParam_DualityGapAbsTolInacc);
      reltol  = this->getStgs().get(SolverDoubleParam_DualityGapRelTolInacc);
    }

    // Optimality
    if ( (cx_<0.0 || by_+hz_<=abstol) && (this->getInfo().get(SolverDoubleParam_PrimalResidual)<feastol && this->getInfo().get(SolverDoubleParam_DualResidual)<feastol) && (this->getInfo().get(SolverDoubleParam_DualityGap)<abstol || this->getInfo().get(SolverDoubleParam_RelativeDualityGap)<reltol)) {
      this->getPrinter().display(Msg::OptimalityReached, this->getInfo());
      (mode==PrecisionConvergence::Full ? exitcode=ExitCode::Optimal : exitcode=ExitCode::OptimalInacc);
    }

    // Dual infeasible
    else if( (this->getInfo().get(SolverDoubleParam_DualInfeasibility)!=SolverSetting::nan) && (this->getInfo().get(SolverDoubleParam_DualInfeasibility)<feastol) && (opt_.tau()<opt_.kappa()) ){
      this->getPrinter().display(Msg::DualInfeasibility, this->getInfo());
      (mode==PrecisionConvergence::Full ? exitcode=ExitCode::DualInf : exitcode=ExitCode::DualInfInacc);
    }

    // Primal infeasible
    else if( ((this->getInfo().get(SolverDoubleParam_PrimalInfeasibility)!=SolverSetting::nan && this->getInfo().get(SolverDoubleParam_PrimalInfeasibility)<feastol) && (opt_.tau()<opt_.kappa())) || (opt_.tau()<feastol && opt_.kappa()<feastol && this->getInfo().get(SolverDoubleParam_PrimalInfeasibility)<feastol)) {
      this->getPrinter().display(Msg::PrimalInfeasibility, this->getInfo());
      (mode==PrecisionConvergence::Full ? exitcode=ExitCode::PrimalInf : exitcode=ExitCode::PrimalInfInacc);
    }
    return exitcode;
  }

  void InteriorPointSolver::saveIterateAsBest()
  {
    best_opt_ = opt_;
    this->getBestInfo() = this->getInfo();
    this->getBestInfo().get(SolverIntParam_NumIter) = this->getInfo().get(SolverIntParam_NumIter);
  }

  void InteriorPointSolver::restoreBestIterate()
  {
	opt_ = best_opt_;
	this->getInfo() = this->getBestInfo();
  }

  void InteriorPointSolver::rhsAffineStep()
  {
	const int* invPerm = this->getLinSolver().invPerm().indices().data();
	for (int i=0; i<this->getCone().numVars(); i++ ) { RHS2_[invPerm[i]] = res_.x()[i]; }
	for (int i=0; i<this->getCone().numLeq(); i++ )  { RHS2_[invPerm[this->getCone().numVars()+i]] = -res_.y()[i]; }
	for (int i=0; i<this->getCone().sizeLpc(); i++ ) { RHS2_[invPerm[this->getCone().lpConeStart()+i]] = opt_.s()[i] - res_.z()[i]; }

	for (int l=0; l<this->getCone().numSoc(); l++) {
	  for (int i=0; i < this->getCone().sizeSoc(l); i++ )
		RHS2_[invPerm[this->getCone().extStartSoc(l)+i]] = opt_.s()[this->getCone().startSoc(l)+i] - res_.z()[this->getCone().startSoc(l)+i];
	}
  }

  void InteriorPointSolver::rhsCenteringPredictorStep()
  {
	double* dz_combined_ptr = dz_combined_.data();
	const int* invPerm = this->getLinSolver().invPerm().indices().data();
	double factor = 1.0 - this->getInfo().get(SolverDoubleParam_CorrectionStepLength);

	for (int i=0; i<this->getCone().numVars(); i++) { RHS2_[invPerm[i]] *= factor; }
	for (int i=0; i<this->getCone().numLeq(); i++ ) { RHS2_[invPerm[this->getCone().numVars()+i]] *= factor; }
    for (int i=0; i<this->getCone().sizeLpc(); i++) { RHS2_[invPerm[this->getCone().lpConeStart()+i]] = dz_combined_ptr[i]; }
	for (int l=0; l < this->getCone().numSoc(); l++ ){
	  for (int i=0; i<this->getCone().sizeSoc(l); i++)
		RHS2_[invPerm[this->getCone().extStartSoc(l)+i]] = dz_combined_ptr[this->getCone().startSoc(l)+i];
	}
  }

  ExitCode InteriorPointSolver::optimize()
  {
	prev_pres_ = SolverSetting::nan;
	exitcode_ = ExitCode::Indeterminate;

	if (initializeVariables() == ExitCode::Indeterminate)
	  return ExitCode::Indeterminate;

	for (this->getInfo().get(SolverIntParam_NumIter)=0; this->getInfo().get(SolverIntParam_NumIter)<=this->getStgs().get(SolverIntParam_SolverMaxIters); this->getInfo().get(SolverIntParam_NumIter)++)
	{
	  computeResiduals();
	  updateStatistics();
	  this->getPrinter().display(Msg::OptimizationProgress, this->getInfo());

	  // SAFEGUARD: Bad update or DualityGap became negative
      if (this->getInfo().get(SolverIntParam_NumIter)>0 && (this->getInfo().get(SolverDoubleParam_PrimalResidual)>this->getStgs().get(SolverDoubleParam_SafeGuard)*prev_pres_ || this->getInfo().get(SolverDoubleParam_DualityGap)<0))
      {
    	this->getPrinter().display(Msg::SearchDirection, best_info_);
        restoreBestIterate();
        exitcode_ = convergenceCheck(PrecisionConvergence::Reduced);
        if (exitcode_ == ExitCode::NotConverged) {
          this->getPrinter().display(Msg::NumericalProblem, info_);
          exitcode_ = ExitCode::PrSearchDirection;
        }
        break;
      }
      prev_pres_ = this->getInfo().get(SolverDoubleParam_PrimalResidual);

      // Check termination to full precision, mininum stepLength and maxNumIters reached
      exitcode_ = convergenceCheck(PrecisionConvergence::Full);
      if (exitcode_ == ExitCode::NotConverged)
      {
        // Mininum stepLength
        if (this->getInfo().get(SolverIntParam_NumIter)>0 && this->getInfo().get(SolverDoubleParam_StepLength)==this->getStgs().get(SolverDoubleParam_MinimumStepLength)*this->getStgs().get(SolverDoubleParam_StepLengthScaling))
        {
          this->getPrinter().display(Msg::LineSearchStagnation, this->getBestInfo());
          restoreBestIterate();
          exitcode_ = convergenceCheck(PrecisionConvergence::Reduced);
          if (exitcode_ == ExitCode::NotConverged) {
            exitcode_ = ExitCode::PrSearchDirection;
            this->getPrinter().display(Msg::NumericalProblem, this->getInfo());
          }
          break;
        }
        // Reached maxNumIters
        else if (this->getInfo().get(SolverIntParam_NumIter)==this->getStgs().get(SolverIntParam_SolverMaxIters))
        {
          if (!this->getInfo().isBetterThan(this->getBestInfo())) { restoreBestIterate(); }
          exitcode_ = convergenceCheck(PrecisionConvergence::Reduced);
          if (exitcode_ == ExitCode::NotConverged) {
            exitcode_ = ExitCode::ReachMaxIters;
            this->getPrinter().display(Msg::MaxItersReached, this->getInfo());
          }
          break;
        }
      } else {
        break;
      }

      // SAFEGUARD: Compare statistics
      if (this->getInfo().get(SolverIntParam_NumIter)==0) { saveIterateAsBest(); }
      else if (this->getInfo().isBetterThan(this->getBestInfo())) { saveIterateAsBest(); }

      // update NTscalings, numeric factorization and solve linear system
	  if (this->getCone().updateNTScalings(this->opt_.s(), this->opt_.z(), this->lambda_)==ConeStatus::Outside) {
		this->getPrinter().display(Msg::VariablesLeavingCone, this->getBestInfo());
		restoreBestIterate();
        exitcode_ = convergenceCheck(PrecisionConvergence::Reduced);
        if (exitcode_ == ExitCode::NotConverged) {
          this->getPrinter().display(Msg::NumericalProblem, this->getInfo());
          return ExitCode::PrSlacksLeaveCone;
        } else { break; }
      }

	  this->getLinSolver().updateMatrix();
	  if (this->getLinSolver().numericFactorization()!=FactStatus::Optimal) {
		this->getPrinter().display(Msg::MatrixFactorization, this->getInfo());
	    return ExitCode::Indeterminate;
	  }
	  this->getInfo().get(SolverIntParam_NumRefsLinSolve) = this->getLinSolver().solve(RHS1_, dopt2_);

	  // Affine Step
	  rhsAffineStep();
	  this->getInfo().get(SolverIntParam_NumRefsLinSolveAffine) = this->getLinSolver().solve(RHS2_, dopt1_);

	  dt_denom_ = opt_.kappa()/opt_.tau() - dotProduct(this->getCone().numVars(), this->getStg().c().data(), dopt2_.x().data()) - dotProduct(this->getCone().numLeq(), this->getStg().b().data(), dopt2_.y().data()) - dotProduct(this->getCone().sizeCone(), this->getStg().h().data(), dopt2_.z().data());
	  dt_affine_ = (residual_t_ - opt_.kappa() + dotProduct(this->getCone().numVars(), this->getStg().c().data(), dopt1_.x().data()) + dotProduct(this->getCone().numLeq(), this->getStg().b().data(), dopt1_.y().data()) + dotProduct(this->getCone().sizeCone(), this->getStg().h().data(), dopt1_.z().data())) / dt_denom_;
	  dk_affine_ = -this->opt_.kappa() - this->opt_.kappa()/this->opt_.tau()*dt_affine_;
	  for (int i=0; i<this->getCone().sizeCone(); i++) { dopt1_.z()[i] += dt_affine_*dopt2_.z()[i]; }

	  W_times_dz_affine_ = this->getCone().W()*dopt1_.z();
	  for (int i=0; i<this->getCone().sizeCone(); i++) { ds_affine_by_W_[i] = -W_times_dz_affine_[i] - lambda_[i]; }
	  this->getInfo().get(SolverDoubleParam_AffineStepLength) = lineSearch(ds_affine_by_W_, W_times_dz_affine_, this->opt_.tau(), dt_affine_, this->opt_.kappa(), dk_affine_);
	  this->getInfo().get(SolverDoubleParam_CorrectionStepLength) = std::max(std::min(std::pow(1.0-this->getInfo().get(SolverDoubleParam_AffineStepLength),3.0),this->getStgs().get(SolverDoubleParam_MaximumCenteringStep)),this->getStgs().get(SolverDoubleParam_MinimumCenteringStep));

	  // Centering and Corrector Step
	  ds_combined_ = lambda_*lambda_ + ds_affine_by_W_*W_times_dz_affine_- (this->getInfo().get(SolverDoubleParam_CorrectionStepLength)*this->getInfo().get(SolverDoubleParam_MeritFunction));
	  ds_affine_by_W_ = lambda_/ds_combined_;
	  dz_combined_ = (this->getInfo().get(SolverDoubleParam_CorrectionStepLength)-1.0)*res_.z() + this->getCone().W()*ds_affine_by_W_;
	  dk_combined_ = this->opt_.kappa()*this->opt_.tau() + dk_affine_*dt_affine_ - this->getInfo().get(SolverDoubleParam_CorrectionStepLength)*this->getInfo().get(SolverDoubleParam_MeritFunction);

	  rhsCenteringPredictorStep();
	  this->getInfo().get(SolverIntParam_NumRefsLinSolveCorrector) = this->getLinSolver().solve(RHS2_, dopt1_);

	  dopt1_.tau() = ((1-this->getInfo().get(SolverDoubleParam_CorrectionStepLength))*this->residual_t_ - dk_combined_/this->opt_.tau() + dotProduct(this->getCone().numVars(), this->getStg().c().data(), dopt1_.x().data()) + dotProduct(this->getCone().numLeq(), this->getStg().b().data(), dopt1_.y().data()) + dotProduct(this->getCone().sizeCone(), this->getStg().h().data(), dopt1_.z().data())) / dt_denom_;
	  dopt1_.xyz() += dopt1_.tau()*dopt2_.xyz();
	  W_times_dz_affine_ = this->getCone().W()*dopt1_.z();
	  for (int i=0; i<this->getCone().sizeCone(); i++) { ds_affine_by_W_[i] = -(ds_affine_by_W_[i] + W_times_dz_affine_[i]); }
	  dopt1_.kappa() = -(dk_combined_ + this->opt_.kappa()*dopt1_.tau())/this->opt_.tau();
	  this->getInfo().get(SolverDoubleParam_StepLength) = lineSearch(ds_affine_by_W_, W_times_dz_affine_, this->opt_.tau(), dopt1_.tau(), this->opt_.kappa(), dopt1_.kappa()) * this->getStgs().get(SolverDoubleParam_StepLengthScaling);
	  dopt1_.s() = this->getCone().W()*ds_affine_by_W_;

	  // Update variables
	  opt_ += this->getInfo().get(SolverDoubleParam_StepLength) * dopt1_;
	}

	this->getEqRoutine().scaleVariables(opt_);
	return exitcode_;
  }

}
