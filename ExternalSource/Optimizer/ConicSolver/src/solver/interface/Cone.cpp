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

#include <solver/interface/Cone.hpp>

namespace solver {

  // Scaling Operator
  ConicVector ScalingOperator::operator*(const Eigen::Ref<const Eigen::VectorXd>& rhs) const
  {
	ConicVector conic_scaling;  conic_scaling.initialize(*(this->cone_));
	this->cone_->conicNTScaling(rhs, conic_scaling.z());
    return conic_scaling;
  }

  // Nesterov Todd scaling class
  NesterovToddScaling::NesterovToddScaling(int conesize)
  {
	wbar_.resize(conesize);
	SOC_id_.resize(3*conesize+1);
  }

  void Cone::initialize(int nvars, int nleq, int nlineq, const Eigen::VectorXi& nsoc)
  {
	neq_ = nleq;
	nvars_ = nvars;
	ssoc_ = nsoc.sum();
	nsoc_ = nsoc.size();
	extssoc_ = ssoc_ + 2*nsoc_;
	lpconestart_ = nvars + nleq;
	soconestart_ = nvars + nleq + nlineq;
	sizecone_ = nlineq + ssoc_;
	extsizecone_ = sizecone_ + 2*nsoc_;
	sizeconstraints_ = nleq + sizecone_;
	sizeproblem_ = nvars + sizeconstraints_;
	extsizeproblem_ = sizeproblem_ + 2*nsoc_;
	this->setupLpcone(nlineq);
	this->setupSocone(nsoc);
	W_scaling_operator_.initialize(*this);
	skbar_.resize(sizecone_);
	zkbar_.resize(sizecone_);
  }

  void Cone::setupLpcone(int size)
  {
	nineq_ = size;
	LP_id_.resize(size);
	LP_scaling_.resize(size);
	LP_sq_scaling_.resize(size);
  }

  // Cone class
  void Cone::setupSocone(const Eigen::VectorXi& indices)
  {
	q_ = indices;
	soc_.clear();
	SOC_start_.resize(nsoc_);
	opt_SOC_start_.resize(nsoc_);
	ext_SOC_start_.resize(nsoc_);
	for (int i=0; i<nsoc_; i++) {
	  soc_.push_back(NesterovToddScaling(indices(i)));
	  SOC_start_(i) = nineq_ + indices.head(i).sum();
	  opt_SOC_start_(i) = nvars_ + neq_ + SOC_start_(i);
	  ext_SOC_start_(i) = nvars_ + neq_ + SOC_start_(i) + 2*i;
	}
  }

  inline double Cone::safeDivision(double x, double y) const
  {
	return ( y<1e-13 ? x/1e-13 : x/y );
  }

  inline double Cone::conicResidual(const double* u, int size) const
  {
	Eigen::Map<const Eigen::VectorXd> eig_u(u+1,size-1);
  	return u[0]*u[0] - eig_u.squaredNorm();
  }

  inline double Cone::conicResidual(const double* u, const double* v, int size) const
  {
  	double result = u[0]*v[0];
  	for (int i=1; i<size; i++) { result -= u[i]*v[i]; }
  	return result;
  }

  double Cone::conicResidual(const Eigen::Ref<const Eigen::VectorXd>& u) const
  {
	return this->conicResidual(u.data(), u.size());
  }

  double Cone::conicResidual(const Eigen::Ref<const Eigen::VectorXd>& u, const Eigen::Ref<const Eigen::VectorXd>& v) const
  {
	return this->conicResidual(u.data(), v.data(), u.size());
  }

  double Cone::conicProduct(const Eigen::Ref<const Eigen::VectorXd> uvec,
		  const Eigen::Ref<const Eigen::VectorXd> vvec, Eigen::Ref<Eigen::VectorXd> wvec) const
  {
	double* w = wvec.data();
	const double* u = uvec.data();
	const double* v = vvec.data();

    // Linear cone
	for (int i=0; i<this->sizeLpc(); i++) { w[i] = u[i]*v[i]; }
    double mu = wvec.head(this->sizeLpc()).cwiseAbs().sum();

	for (int i=0; i<this->numSoc(); i++) {
	  w[this->startSoc(i)] = 0.0;
	  for (int j=0; j<this->sizeSoc(i); j++)
		w[this->startSoc(i)] += u[this->startSoc(i)+j]*v[this->startSoc(i)+j];

	  for (int j=1; j<this->sizeSoc(i); j++)
		w[this->startSoc(i)+j] = u[this->startSoc(i)]*v[this->startSoc(i)+j] + v[this->startSoc(i)]*u[this->startSoc(i)+j];

      mu += std::abs(w[this->startSoc(i)]);
	}
    return mu;
  }

  void Cone::conicDivision(const Eigen::Ref<const Eigen::VectorXd>& uvec, const Eigen::Ref<const Eigen::VectorXd>& wvec, Eigen::Ref<Eigen::VectorXd> vvec) const
  {
	double* v = vvec.data();
	const double* u = uvec.data();
	const double* w = wvec.data();

    // Linear cone
	for (int i=0; i<this->sizeLpc(); i++) { v[i] = this->safeDivision(w[i],u[i]); }

    // Second-order cone
	double rho, uwn, factor;
	for (int i=0; i<this->numSoc(); i++) {
	  rho = conicResidual(u+this->startSoc(i),this->sizeSoc(i));
	  uwn = 0; for (int j=1; j<this->sizeSoc(i); j++) { uwn += u[this->startSoc(i)+j]*w[this->startSoc(i)+j]; }
      factor = this->safeDivision(this->safeDivision(uwn,u[this->startSoc(i)])-w[this->startSoc(i)],rho);
	  v[this->startSoc(i)] = this->safeDivision(u[this->startSoc(i)]*w[this->startSoc(i)]-uwn,rho);
	  for (int j=1; j<this->sizeSoc(i); j++)
		v[this->startSoc(i)+j] = factor*u[this->startSoc(i)+j] + this->safeDivision(w[this->startSoc(i)+j], u[this->startSoc(i)]);
	}
  }

  void Cone::conicProjection(Eigen::Ref<Eigen::VectorXd> s)
  {
	// Find maximum residual
	double* s_ptr = s.data();
	double max_residual = 0.0;
	for (int i=0; i<this->sizeLpc(); i++) { if (s(i) < 0.0) { max_residual = std::max(max_residual,fabs(s(i))); } }
	for (int i=0; i<this->numSoc(); i++) {
	  double residual = 0.0;
	  for (int j=0; j<this->sizeSoc(i)-1; j++) { residual += std::pow(s_ptr[this->startSoc(i)+1+j], 2.0); }
	  residual = std::max(std::sqrt(residual)-s_ptr[this->startSoc(i)],0.0);
      if (residual > max_residual) { max_residual = residual; }
    }

    // Scale variables appropriately into the cone
    s.head(this->sizeLpc()).array() += max_residual + 1.0;
    for (int i=0; i<this->numSoc(); i++)
      s_ptr[this->startSoc(i)] += max_residual + 1.0;
  }

  void Cone::conicNTScaling(const Eigen::VectorXd& z, Eigen::Ref<Eigen::VectorXd> lambda) const
  {
	this->conicNTScaling(z.data(), lambda.data());
  }

  void Cone::conicNTScaling(const double* z, double* lambda) const
  {
  	// Linear cone
  	for (int i=0; i<this->sizeLpc(); i++){ lambda[i] = this->scalingLpc(i) * z[i]; }

  	// Second order cone
  	for (int l=0; l<this->numSoc(); l++) {
  	  int conesize = this->sizeSoc(l);
  	  int conestart = this->startSoc(l);
  	  Eigen::Map<const Eigen::VectorXd> eig_z1(z+conestart+1, conesize-1);

  	  double lzero = this->soc(l).scalingSoc().tail(conesize-1).dot(eig_z1);
  	  double factor = z[conestart] + this->safeDivision(lzero,(1.0+this->soc(l).scalingSoc(0)));

  	  lambda[conestart] = this->soc(l).eta()*(this->soc(l).scalingSoc(0)*z[conestart] + lzero);
  	  for (int i=1; i<this->sizeSoc(l); i++)
  		lambda[conestart+i] = this->soc(l).eta()*(z[conestart+i] + factor*this->soc(l).scalingSoc(i));
  	}
  }

  void Cone::conicNTScaling2(const Eigen::VectorXd& xvec, Eigen::Ref<Eigen::VectorXd> yvec) const
  {
	double* y = yvec.data();
	const double* x = xvec.data(), *scaling_soc;

	// Linear cone
	const double* sq_scaling_lpc = this->sqScalingLpc().data();
	for (int i=0; i<this->sizeLpc(); i++) { y[i] += sq_scaling_lpc[i] * x[i]; }

	// Second order cone
	int conesize, conestart;
	for (int l=0; l<this->numSoc(); l++) {
	  conesize = this->sizeSoc(l);
	  conestart = this->startSoc(l) + 2*l;
	  scaling_soc = this->soc(l).scalingSoc().data();

	  y[conestart] += this->soc(l).etaSquare()*(this->soc(l).d1()*x[conestart] + this->soc(l).u0()*x[conestart+conesize+1]);
	  double yaux1  = this->soc(l).v1()*x[conestart+conesize] + this->soc(l).u1()*x[conestart+conesize+1];
	  double yaux2  = 0.0;
	  for (int i=1; i<conesize; i++) {
	    y[conestart+i] += this->soc(l).etaSquare()*(x[conestart+i] + yaux1*scaling_soc[i]);
	    yaux2 += scaling_soc[i]*x[conestart+i];
	  }

	  y[conestart+conesize  ] += this->soc(l).etaSquare()*(this->soc(l).v1()*yaux2 + x[conestart+conesize]);
	  y[conestart+conesize+1] += this->soc(l).etaSquare()*(this->soc(l).u0()*x[conestart] + this->soc(l).u1()*yaux2 - x[conestart+conesize+1]);
	}
  }

  void Cone::unpermuteSolution(const Eigen::PermutationMatrix<Eigen::Dynamic,Eigen::Dynamic>& Perm,
                               const Eigen::VectorXd& Px, OptimizationVector& sd) const
  {
	double* dx = sd.x().data();
	double* dy = sd.y().data();
	double* dz = sd.z().data();
	const int* perm = Perm.indices().data();

	int k=this->soConeStart(), j=this->sizeLpc();
    for (int i=0; i<this->numVars(); i++) { dx[i] = Px[perm[i]]; }
    for (int i=0; i<this->numLeq(); i++)  { dy[i] = Px[perm[i+this->numVars()]]; }
    for (int i=0; i<this->sizeLpc(); i++) { dz[i] = Px[perm[i+this->lpConeStart()]]; }
    for (int l=0; l<this->numSoc(); l++) {
      for (int i=0; i<this->sizeSoc(l); i++)
        dz[j++] = Px[perm[k++]];
      k += 2;
    }
  }

  void Cone::unpermuteSolution(const Eigen::PermutationMatrix<Eigen::Dynamic,Eigen::Dynamic>& Perm,
                               const Eigen::VectorXd& Px, OptimizationVector& sd, Eigen::VectorXd& permdZ) const
  {
	double* dx = sd.x().data();
	double* dy = sd.y().data();
	double* dz = sd.z().data();
	const int* perm = Perm.indices().data();

	int k=this->soConeStart(), j=this->sizeLpc();
    for (int i=0; i<this->numVars(); i++) { dx[i] = Px[perm[i]]; }
    for (int i=0; i<this->numLeq(); i++)  { dy[i] = Px[perm[i+this->numVars()]]; }
    for (int i=0; i<this->sizeLpc(); i++) { dz[i] = Px[perm[i+this->lpConeStart()]]; }
    for (int l=0; l<this->numSoc(); l++) {
      for (int i=0; i<this->sizeSoc(l); i++)
        dz[j++] = Px[perm[k++]];
      k += 2;
    }
    for (int i=0; i<this->extSizeCone(); i++) { permdZ[i] = Px[perm[i+this->lpConeStart()]]; }
  }

  ConeStatus Cone::updateNTScalings(const Eigen::VectorXd& svec, const Eigen::VectorXd& zvec, Eigen::VectorXd& lambda)
  {
	const double* s = svec.data();
	const double* z = zvec.data();
	double sresidual, zresidual, w0, wn, alpha, beta;

  	// Linear cone
  	for (int i=0; i<this->sizeLpc(); i++) {
  	  this->sqScalingLpc(i) = this->safeDivision(s[i], z[i]);
  	  this->scalingLpc(i) = std::sqrt(this->sqScalingLpc(i));
  	}

	// Second order cone
  	for (int l=0; l<this->numSoc(); l++) {
  	  int conesize = this->sizeSoc(l);
  	  int conestart = this->startSoc(l);

  	  // check residuals
	  sresidual = this->conicResidual(s+conestart, conesize);
	  zresidual = this->conicResidual(z+conestart, conesize);
	  if ( sresidual<=0 || zresidual<=0 ) { return ConeStatus::Outside; }

	  // normalize variables
	  Eigen::Map<Eigen::VectorXd> skbar(&skbar_[conestart], conesize);
	  Eigen::Map<Eigen::VectorXd> zkbar(&zkbar_[conestart], conesize);
	  double sresidual_inv = this->safeDivision(1.0, std::sqrt(sresidual));
	  double zresidual_inv = this->safeDivision(1.0, std::sqrt(zresidual));
	  for (int i=0; i<conesize; i++) { skbar[i] = s[i+conestart] * sresidual_inv; }
	  for (int i=0; i<conesize; i++) { zkbar[i] = z[i+conestart] * zresidual_inv; }
	  this->soc(l).etaSquare() = this->safeDivision(std::sqrt(sresidual), std::sqrt(zresidual));
	  this->soc(l).eta() = sqrt(this->soc(l).etaSquare());

	  // computing Nesterov-Todd scaling point
	  double normalizer = this->safeDivision(0.5,std::sqrt(0.5 + 0.5*skbar.dot(zkbar)));
	  this->soc(l).scalingSoc(0) = normalizer*(skbar[0] + zkbar[0]);
	  this->soc(l).w() = 0.0;
	  for (int i=1; i<conesize; i++) {
		this->soc(l).scalingSoc(i) = normalizer*(skbar[i] - zkbar[i]);
		this->soc(l).w() += std::pow(this->soc(l).scalingSoc(i), 2.0);
	  }

	  // computing variables for KKT matrix update
	  w0 = this->soc(l).scalingSoc(0);
	  wn = this->soc(l).scalingSoc().tail(conesize-1).squaredNorm();
	  alpha = 1.0 + w0 + this->safeDivision(wn, 1.0 + w0);
	  beta = 1.0 + this->safeDivision(2.0, 1.0 + w0) + this->safeDivision(wn,std::pow(1.0 + w0, 2.0));
	  this->soc(l).d1() = std::max(0.5*(w0*w0 + wn*(1.0 - this->safeDivision(alpha*alpha,1.0 + wn*beta) )), 0.0);
	  this->soc(l).u0() = std::sqrt(w0*w0+wn-this->soc(l).d1());
	  this->soc(l).u1() = this->safeDivision(alpha,this->soc(l).u0());
	  this->soc(l).v1() = std::pow(this->soc(l).u1(),2.0) - beta;
	  if (this->soc(l).v1() <= 0) { return ConeStatus::Outside; }
	  this->soc(l).v1() = std::sqrt(this->soc(l).v1());
	}
	this->conicNTScaling(zvec, lambda);

	return ConeStatus::Inside;
  }

  // Vector class
  void Vector::initialize(const Cone& cone)
  {
	cone_ = &cone;
    this->resize(cone_->sizeProb());
    this->setZero();
  }

  // ConicVector class
  void ConicVector::initialize(const Cone& cone)
  {
	cone_ = &cone;
	this->resize(cone_->sizeCone());
	this->setZero();
  }

  ConicVector ConicVector::operator/(const ConicVector& rhs) const
  {
	ConicVector conic_division;  conic_division.initialize(*(this->cone_));
	this->cone_->conicDivision(this->z(), rhs.z(), conic_division.z());
    return conic_division;
  }

  ConicVector ConicVector::operator*(const ConicVector& rhs) const
  {
	ConicVector conic_product;  conic_product.initialize(*(this->cone_));
	this->cone_->conicProduct(this->z(), rhs.z(), conic_product.z());
    return conic_product;
  }

  ConicVector ConicVector::operator+(const ConicVector& rhs) const
  {
	ConicVector conic_sum;  conic_sum.initialize(*(this->cone_));
	conic_sum.z() = this->z() + rhs.z();
    return conic_sum;
  }

  ConicVector ConicVector::operator+(const double& rhs) const
  {
	ConicVector conic_sum;  conic_sum.initialize(*(this->cone_));
	conic_sum.z() = this->z();
	conic_sum.zLpc().array() += rhs;
    for (int i=0; i<this->cone_->numSoc(); i++)
      conic_sum.zSoc(i)[0] += rhs;
    return conic_sum;
  }

  ConicVector ConicVector::operator-(const double& rhs) const
  {
	ConicVector conic_diff;  conic_diff.initialize(*(this->cone_));
	conic_diff.z() = this->z();
	conic_diff.zLpc().array() -= rhs;
    for (int i=0; i<this->cone_->numSoc(); i++)
      conic_diff.zSoc(i)[0] -= rhs;
    return conic_diff;
  }

  ConicVector& ConicVector::operator+=(const double& rhs)
  {
    this->zLpc().array() += rhs;
    for (int i=0; i<this->cone_->numSoc(); i++)
	  this->zSoc(i)[0] += rhs;
    return *this;
  }

  ConicVector ConicVector::operator-() const
  {
	ConicVector conic_vec;  conic_vec.initialize(*(this->cone_));
	conic_vec.z() = -this->z();
    return conic_vec;
  }

  // ExtendedVector class
  void ExtendedVector::initialize(const Cone& cone)
  {
	cone_ = &cone;
	this->resize(cone_->extSizeProb());
	this->setZero();
  }

  void ExtendedVector::zRtoE(const Eigen::Ref<const Eigen::VectorXd>& rvec)
  {
	this->zLpc() = rvec.head(cone_->sizeLpc());
	for (int i=0; i<cone_->numSoc(); i++) {
	  this->zSoc(i).tail(2).setZero();
	  this->zSoc(i).head(cone_->sizeSoc(i)) = rvec.segment(cone_->startSoc(i),cone_->sizeSoc(i));
	}
  }

  // OptimizationVector class
  void OptimizationVector::initialize(const Cone& cone)
  {
	cone_ = &cone;
	this->resize(cone_->sizeProb()+cone_->sizeLpc()+cone_->sizeSoc()+2);
	this->setZero();
  }

  OptimizationVector OptimizationVector::operator+(const OptimizationVector& rhs) const
  {
	OptimizationVector result = *this;
	result += rhs;
	return result;
  }

  // SolverStorage
  void SolverStorage::initialize(Cone& cone, SolverSetting& stgs)
  {
	stgs_ = &stgs;
	cone_ = &cone;
    cbh_.initialize(cone);
    u_opt_.initialize(cone);
    v_opt_.initialize(cone);
    u_t_opt_.initialize(cone);
    cbh_copy_.initialize(cone);
    u_prev_opt_.initialize(cone);
    this->A().resize(cone_->numLeq(), cone_->numVars());
    this->G().resize(cone_->sizeLpc() + cone_->sizeSoc(), cone_->numVars());
  }

  void SolverStorage::addCoeff(const Eigen::Triplet<double>& coeff, bool flag_eq)
  {
	if (flag_eq == true) { this->Acoeffs().push_back(coeff); }
	else { this->Gcoeffs().push_back(coeff); }
  }

  void SolverStorage::initializeMatrices()
  {
	this->A().setFromTriplets(Acoeffs_.begin(), Acoeffs_.end());
	if (!this->A().isCompressed()) { this->A().makeCompressed(); }

	this->G().setFromTriplets(Gcoeffs_.begin(), Gcoeffs_.end());
	if (!this->G().isCompressed()) { this->G().makeCompressed(); }
  }

}
