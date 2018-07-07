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

#pragma once

#include <memory>
#include <vector>
#include <iostream>
#include <Eigen/Sparse>
#include <solver/interface/SolverSetting.hpp>

namespace solver {

  class Cone;
  class Vector;
  class ConicVector;
  class ExtendedVector;
  class OptimizationVector;

  class ScalingOperator
  {
    public:
	  ScalingOperator(){}
	  ~ScalingOperator(){}

	  void initialize(const Cone& cone) { cone_ = &cone; }
	  ConicVector operator*(const Eigen::Ref<const Eigen::VectorXd>& rhs) const;

    private:
	  const Cone* cone_;
  };

  /**
   * This class performs appropriate scalings of the cone variables.
   * Details can be found in the paper: Domahidi, A. and Chu, E. and Boyd, S.,
   *     ECOS: An SOCP solver for embedded systems, ECC 2013, pages 3071-3076
   */
  class NesterovToddScaling
  {
    public:
	  friend class Cone;

	  NesterovToddScaling(int conesize);
	  ~NesterovToddScaling(){}

	  inline double& w() { return w_; }
	  inline double& d1() { return d1_; }
	  inline double& u0() { return u0_; }
	  inline double& u1() { return u1_; }
	  inline double& v1() { return v1_; }
	  inline double& eta() { return eta_; }
	  inline double& etaSquare() { return eta_square_; }
	  int& indexSoc(int id) { return SOC_id_(id); }
	  Eigen::VectorXd& scalingSoc() { return wbar_; }
	  double& scalingSoc(int id) { return wbar_(id); }

	  inline const double& w() const { return w_; }
	  inline const double& d1() const { return d1_; }
	  inline const double& u0() const { return u0_; }
	  inline const double& u1() const { return u1_; }
	  inline const double& v1() const { return v1_; }
	  inline const double& eta() const { return eta_; }
	  inline const double& etaSquare() const { return eta_square_; }
	  const int& indexSoc(int id) const { return SOC_id_(id); }
	  const Eigen::VectorXd& scalingSoc() const { return wbar_; }
	  const double& scalingSoc(int id) const { return wbar_(id); }

    private:
	  Eigen::VectorXd wbar_;
	  Eigen::VectorXi SOC_id_;
	  double w_, d1_, u0_, u1_, v1_, eta_, eta_square_;
  };

  /**
   * This class contains all information about the conic optimization
   * problem, and provides functionality for conic operations, such as
   * scalings, residuals and sizes.
   */
  class Cone
  {
    public:
	  // Cone class
	  Cone(){}
	  ~Cone(){}

	  // Setting up problem
	  void setupLpcone(int size);
	  void setupSocone(const Eigen::VectorXi& indices);
	  void initialize(int nvars, int nleq, int nlineq, const Eigen::VectorXi& nsoc);

	  // Getter and setter methods for problem size variables
	  inline const int numLeq() const { return neq_; }
	  inline const int numSoc() const { return nsoc_; }
	  inline const int sizeSoc() const { return ssoc_; }
	  inline const int sizeLpc() const { return nineq_; }
	  inline const int numVars() const { return nvars_; }
	  inline const int sizeCone() const { return sizecone_; }
	  inline const int extSizeSoc() const { return extssoc_; }
	  inline const int sizeSoc(int id) const { return q_(id); }
	  inline const int numCones() const { return nineq_+nsoc_; }
	  inline const int sizeProb() const { return sizeproblem_; }
	  inline const int extSizeCone() const { return extsizecone_; }
	  inline const int lpConeStart() const { return lpconestart_; }
	  inline const int soConeStart() const { return soconestart_; }
	  inline const int extSizeProb() const { return extsizeproblem_; }
	  inline const int startSoc(int id) const { return SOC_start_(id); }
	  inline const int sizeConstraints() const { return sizeconstraints_; }
	  inline const int optStartSoc(int id) const { return opt_SOC_start_(id); }
	  inline const int extStartSoc(int id) const { return ext_SOC_start_(id); }
	  static double normSoc(const Eigen::Ref<const Eigen::VectorXd>& v) { return v[0]-v.tail(v.size()-1).norm(); }

	  // Getter and setter methods for variables of linear cone
	  int& indexLpc(int id) { return LP_id_(id); }
	  Eigen::VectorXd& scalingLpc() { return LP_scaling_; }
	  double& scalingLpc(int id) { return LP_scaling_(id); }
	  Eigen::VectorXd& sqScalingLpc() { return LP_sq_scaling_; }
	  double& sqScalingLpc(int id) { return LP_sq_scaling_(id); }

	  const int& indexLpc(int id) const { return LP_id_(id); }
	  const Eigen::VectorXd& scalingLpc() const { return LP_scaling_; }
	  const double& scalingLpc(int id) const { return LP_scaling_(id); }
	  const Eigen::VectorXd& sqScalingLpc() const { return LP_sq_scaling_; }
	  const double& sqScalingLpc(int id) const { return LP_sq_scaling_(id); }

	  // Getter and setter methods for variables of second order cone
	  NesterovToddScaling& soc(int id) { return soc_[id]; }
	  const NesterovToddScaling& soc(int id) const { return soc_[id]; }
	  inline double conicResidual(const double* u, int size) const;
	  inline double conicResidual(const double* u, const double* v, int size) const;
	  double conicResidual(const Eigen::Ref<const Eigen::VectorXd>& u) const;
	  double conicResidual(const Eigen::Ref<const Eigen::VectorXd>& u, const Eigen::Ref<const Eigen::VectorXd>& v) const;

	  // Functions for cones
	  double safeDivision(double x, double y) const;
	  void conicProjection(Eigen::Ref<Eigen::VectorXd> s);
	  void conicNTScaling(const double* z, double* lambda) const;
	  void conicNTScaling(const Eigen::VectorXd& z, Eigen::Ref<Eigen::VectorXd> lambda) const;
	  void conicNTScaling2(const Eigen::VectorXd& x, Eigen::Ref<Eigen::VectorXd> y) const;
	  void conicDivision(const Eigen::Ref<const Eigen::VectorXd>& u, const Eigen::Ref<const Eigen::VectorXd>& w, Eigen::Ref<Eigen::VectorXd> v) const;
	  double conicProduct(const Eigen::Ref<const Eigen::VectorXd> u, const Eigen::Ref<const Eigen::VectorXd> v, Eigen::Ref<Eigen::VectorXd> w) const;
	  ConeStatus updateNTScalings(const Eigen::VectorXd& s, const Eigen::VectorXd& z, Eigen::VectorXd& lambda);
	  void unpermuteSolution(const Eigen::PermutationMatrix<Eigen::Dynamic,Eigen::Dynamic>& Pinv, const Eigen::VectorXd& Px, OptimizationVector& sd) const;
	  void unpermuteSolution(const Eigen::PermutationMatrix<Eigen::Dynamic,Eigen::Dynamic>& Pinv, const Eigen::VectorXd& Px, OptimizationVector& sd, Eigen::VectorXd& dz) const;

	  // Getter and setter methods for scaling operators
	  ScalingOperator& W() { return W_scaling_operator_; }
	  const ScalingOperator& W() const { return W_scaling_operator_; }

    private:
	  int nvars_;           // number variables
	  int neq_;             // number linear equality constraints
	  int nineq_;           // number linear inequality constraints
	  int nsoc_;            // number second order cone constraints
	  int ssoc_;            // size second order cone constraints
	  int extssoc_;         // extended size second order cone constraints
	  int lpconestart_;     // offset to linear cone start
	  int soconestart_;     // offset to second order cone start
	  int sizecone_;        // size of linear and second order cones
	  int extsizecone_;     // extended size of linear and second order cones
	  int sizeconstraints_; // size of the constraints
	  int sizeproblem_;     // size of number of variables plus constraints
	  int extsizeproblem_;  // size of number of variables plus constraints
	  Eigen::VectorXi q_;   // sizes of second-order cone constraints

	  // variables of LP cone
	  Eigen::VectorXi LP_id_;
	  Eigen::VectorXd LP_scaling_;
	  Eigen::VectorXd LP_sq_scaling_;

	  // variables of SO cone
	  Eigen::VectorXd skbar_, zkbar_;
	  std::vector<NesterovToddScaling> soc_;
	  Eigen::VectorXi SOC_start_, opt_SOC_start_, ext_SOC_start_;

	  // Operators
	  ScalingOperator W_scaling_operator_;
  };

  /**
   * Helper class to work with optimization variables, lay down in normal order.
   */
  class Vector : public Eigen::VectorXd
  {
    public:
	  Vector() : Eigen::VectorXd() {}
	  ~Vector(){}

	  template<typename OtherDerived>
	  Vector(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::VectorXd(other) {}

	  template<typename OtherDerived>
	  Vector& operator=(const Eigen::MatrixBase <OtherDerived>& other) {
	    this->Eigen::VectorXd::operator=(other);
	    return *this;
	  }

	  void initialize(const Cone& cone);

	  Eigen::Ref<Eigen::VectorXd> x() { return this->head(cone_->numVars()); }
	  Eigen::Ref<Eigen::VectorXd> y() { return this->segment(cone_->numVars(), cone_->numLeq()); }
	  Eigen::Ref<Eigen::VectorXd> z() { return this->segment(cone_->lpConeStart(), cone_->sizeCone()); }
	  Eigen::Ref<Eigen::VectorXd> yz() { return this->segment(cone_->numVars(), cone_->sizeConstraints()); }

	  Eigen::Ref<Eigen::VectorXd> zLpc() { return this->segment(cone_->lpConeStart(), cone_->sizeLpc()); }
	  Eigen::Ref<Eigen::VectorXd> zSoc() { return this->segment(cone_->soConeStart(), cone_->sizeSoc()); }
	  Eigen::Ref<Eigen::VectorXd> zSoc(int id) { return this->segment(cone_->optStartSoc(id), cone_->sizeSoc(id)); }

	  const Eigen::Ref<const Eigen::VectorXd> x() const { return this->head(cone_->numVars()); }
	  const Eigen::Ref<const Eigen::VectorXd> y() const { return this->segment(cone_->numVars(), cone_->numLeq()); }
	  const Eigen::Ref<const Eigen::VectorXd> z() const { return this->segment(cone_->lpConeStart(), cone_->sizeCone()); }
	  const Eigen::Ref<const Eigen::VectorXd> yz() const { return this->segment(cone_->numVars(), cone_->sizeConstraints()); }

	  const Eigen::Ref<const Eigen::VectorXd> zLpc() const { return this->segment(cone_->lpConeStart(), cone_->sizeLpc()); }
	  const Eigen::Ref<const Eigen::VectorXd> zSoc() const { return this->segment(cone_->soConeStart(), cone_->sizeSoc()); }
	  const Eigen::Ref<const Eigen::VectorXd> zSoc(int id) const { return this->segment(cone_->optStartSoc(id), cone_->sizeSoc(id)); }

    private:
	  const Cone* cone_;
  };

  /**
   * Helper class to work with variables, which are members of a proper convex cone.
   */
  class ConicVector : public Eigen::VectorXd
  {
    public:
	  ConicVector() : Eigen::VectorXd() {}
	  ConicVector(int size) : Eigen::VectorXd(size) {}
	  ~ConicVector(){}

	  template<typename OtherDerived>
	  ConicVector(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::VectorXd(other) {}

	  template<typename OtherDerived>
	  ConicVector& operator=(const Eigen::MatrixBase <OtherDerived>& other) {
	    this->Eigen::VectorXd::operator=(other);
	    return *this;
	  }

	  ConicVector& operator=(const ConicVector& other) {
	    this->Eigen::VectorXd::operator=(other);
	    this->cone_ = other.cone_;
	    return *this;
	  }

	  void initialize(const Cone& cone);
	  ConicVector  operator/ (const ConicVector& rhs) const;
	  ConicVector  operator* (const ConicVector& rhs) const;
	  ConicVector  operator+ (const ConicVector& rhs) const;
	  ConicVector  operator+ (const double& rhs) const;
	  ConicVector  operator- (const double& rhs) const;
	  ConicVector& operator+=(const double& rhs);
	  ConicVector  operator- () const;

	  Eigen::Ref<Eigen::VectorXd> z() { return this->head(cone_->sizeCone()); }
	  Eigen::Ref<Eigen::VectorXd> zLpc() { return this->head(cone_->sizeLpc()); }
	  Eigen::Ref<Eigen::VectorXd> zSoc() { return this->segment(cone_->sizeLpc(),cone_->sizeSoc()); }
	  Eigen::Ref<Eigen::VectorXd> zSoc(int id) { return this->segment(cone_->startSoc(id),cone_->sizeSoc(id)); }

	  const Eigen::Ref<const Eigen::VectorXd> z() const { return this->head(cone_->sizeCone()); }
	  const Eigen::Ref<const Eigen::VectorXd> zLpc() const { return this->head(cone_->sizeLpc()); }
	  const Eigen::Ref<const Eigen::VectorXd> zSoc() const { return this->segment(cone_->sizeLpc(),cone_->sizeSoc()); }
	  const Eigen::Ref<const Eigen::VectorXd> zSoc(int id) const { return this->segment(cone_->startSoc(id),cone_->sizeSoc(id)); }

    private:
	  const Cone* cone_;
  };

  /**
   * Helper class to work with optimization variables, lay down in an extended order.
   */
  class ExtendedVector : public Eigen::VectorXd
  {
    public:
	  ExtendedVector() : Eigen::VectorXd() {}
	  ~ExtendedVector(){}

	  template<typename OtherDerived>
	  ExtendedVector(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::VectorXd(other) {}

	  template<typename OtherDerived>
	  ExtendedVector& operator=(const Eigen::MatrixBase <OtherDerived>& other) {
	    this->Eigen::VectorXd::operator=(other);
	    return *this;
	  }

	  void initialize(const Cone& cone);

	  void zRtoE(const Eigen::Ref<const Eigen::VectorXd>& rvec);
	  Eigen::Ref<Eigen::VectorXd> x() { return this->head(cone_->numVars()); }
	  Eigen::Ref<Eigen::VectorXd> y() { return this->segment(cone_->numVars(),cone_->numLeq()); }
	  Eigen::Ref<Eigen::VectorXd> z() { return this->segment(cone_->lpConeStart(),cone_->extSizeCone()); }

	  Eigen::Ref<Eigen::VectorXd> zLpc() { return this->segment(cone_->lpConeStart(),cone_->sizeLpc()); }
	  Eigen::Ref<Eigen::VectorXd> zSoc() { return this->segment(cone_->soConeStart(),cone_->extSizeSoc()); }
	  Eigen::Ref<Eigen::VectorXd> zSoc(int id) { return this->segment(cone_->extStartSoc(id),cone_->sizeSoc(id)+2); }

	  const Eigen::Ref<const Eigen::VectorXd> x() const { return this->head(cone_->numVars()); }
	  const Eigen::Ref<const Eigen::VectorXd> y() const { return this->segment(cone_->numVars(),cone_->numLeq()); }
	  const Eigen::Ref<const Eigen::VectorXd> z() const { return this->segment(cone_->lpConeStart(),cone_->extSizeCone()); }

	  const Eigen::Ref<const Eigen::VectorXd> zLpc() const { return this->segment(cone_->lpConeStart(),cone_->sizeLpc()); }
	  const Eigen::Ref<const Eigen::VectorXd> zSoc() const { return this->segment(cone_->soConeStart(),cone_->extSizeSoc()); }
	  const Eigen::Ref<const Eigen::VectorXd> zSoc(int id) const { return this->segment(cone_->extStartSoc(id),cone_->sizeSoc(id)+2); }

    private:
	  const Cone* cone_;
  };

  /**
   * Helper class to define an optimization vector, including primal and dual
   * variables, and variables to render the optimization problem homogeneous.
   */
  class OptimizationVector : public Eigen::VectorXd
  {
    public:
	  OptimizationVector() : Eigen::VectorXd() {}
	  ~OptimizationVector(){}

	  template<typename OtherDerived>
	  OptimizationVector(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::VectorXd(other) {}

	  template<typename OtherDerived>
	  OptimizationVector& operator=(const Eigen::MatrixBase <OtherDerived>& other) {
	    this->Eigen::VectorXd::operator=(other);
	    this->cone_ = other.cone_;
	    return *this;
	  }

	  void initialize(const Cone& cone);

	  OptimizationVector operator+(const OptimizationVector& rhs) const;

	  Eigen::Ref<Eigen::VectorXd> x() { return this->head(cone_->numVars()); }
	  Eigen::Ref<Eigen::VectorXd> y() { return this->segment(cone_->numVars(),cone_->numLeq()); }
	  Eigen::Ref<Eigen::VectorXd> z() { return this->segment(cone_->lpConeStart(),cone_->sizeCone()); }
	  Eigen::Ref<Eigen::VectorXd> s() { return this->segment(cone_->sizeProb()+2,cone_->sizeCone()); }
	  Eigen::Ref<Eigen::VectorXd> yz() { return this->segment(cone_->numVars(),cone_->sizeConstraints()); }
	  Eigen::Ref<Eigen::VectorXd> yztau() { return this->segment(cone_->numVars(),cone_->sizeConstraints()+1); }
	  double& tau() { return this->segment<2>(cone_->sizeProb())[0]; }
	  double& kappa() { return this->segment<2>(cone_->sizeProb())[1]; }

	  const Eigen::Ref<const Eigen::VectorXd> x() const { return this->head(cone_->numVars()); }
	  const Eigen::Ref<const Eigen::VectorXd> y() const { return this->segment(cone_->numVars(),cone_->numLeq()); }
	  const Eigen::Ref<const Eigen::VectorXd> s() const { return this->segment(cone_->lpConeStart(),cone_->sizeCone()); }
	  const Eigen::Ref<const Eigen::VectorXd> z() const { return this->segment(cone_->sizeProb()+2,cone_->sizeCone()); }
	  const Eigen::Ref<const Eigen::VectorXd> yz() const { return this->segment(cone_->numVars(),cone_->sizeConstraints()); }
	  const Eigen::Ref<const Eigen::VectorXd> yztau() const { return this->segment(cone_->numVars(),cone_->sizeConstraints()+1); }
	  double tau() const { return this->segment<2>(cone_->sizeProb())[0]; }
	  double kappa() const { return this->segment<2>(cone_->sizeProb())[1]; }

	  Eigen::Ref<Eigen::VectorXd> xyz() { return this->head(cone_->sizeProb()); }
	  Eigen::Ref<Eigen::VectorXd> xyztau() { return this->head(cone_->sizeProb()+1); }
	  Eigen::Ref<Eigen::VectorXd> zLpc() { return this->segment(cone_->lpConeStart(),cone_->sizeLpc()); }
	  Eigen::Ref<Eigen::VectorXd> zSoc() { return this->segment(cone_->soConeStart(),cone_->sizeSoc()); }
	  Eigen::Ref<Eigen::VectorXd> zSoc(int id) { return this->segment(cone_->optStartSoc(id),cone_->sizeSoc(id)); }

	  const Eigen::Ref<const Eigen::VectorXd> xyz() const { return this->head(cone_->sizeProb()); }
	  const Eigen::Ref<const Eigen::VectorXd> xyztau() const { return this->head(cone_->sizeProb()+1); }
	  const Eigen::Ref<const Eigen::VectorXd> zLpc() const { return this->segment(cone_->lpConeStart(),cone_->sizeLpc()); }
	  const Eigen::Ref<const Eigen::VectorXd> zSoc() const { return this->segment(cone_->soConeStart(),cone_->sizeSoc()); }
	  const Eigen::Ref<const Eigen::VectorXd> zSoc(int id) const { return this->segment(cone_->optStartSoc(id),cone_->sizeSoc(id)); }

    private:
	  const Cone* cone_;
  };

  /**
   * Class that provides storage space for the optimization matrices,
   * vectors and variables.
   */
  class SolverStorage
  {
    public:
	  SolverStorage(){}
	  ~SolverStorage(){}

	  double& gTh() { return gTh_; }
	  const double& gTh() const { return gTh_; }

	  Eigen::SparseMatrix<double>& A() { return A_; }
	  Eigen::SparseMatrix<double>& G() { return G_; }
	  Eigen::SparseMatrix<double>& At() { return At_; }
	  Eigen::SparseMatrix<double>& Gt() { return Gt_; }
	  const Eigen::SparseMatrix<double>& A() const { return A_; }
	  const Eigen::SparseMatrix<double>& G() const { return G_; }
	  const Eigen::SparseMatrix<double>& At() const { return At_; }
	  const Eigen::SparseMatrix<double>& Gt() const { return Gt_; }

	  void initializeMatrices();
	  void initialize(Cone& cone, SolverSetting& stgs);
	  void cleanCoeffs() { Acoeffs_.clear(); Gcoeffs_.clear(); }
	  void addCoeff(const Eigen::Triplet<double>& coeff, bool flag_eq = false);

	  Eigen::Ref<Eigen::VectorXd> cbh() { return cbh_; }
	  Eigen::Ref<Eigen::VectorXd> c() { return cbh_.x(); }
	  Eigen::Ref<Eigen::VectorXd> b() { return cbh_.y(); }
	  Eigen::Ref<Eigen::VectorXd> h() { return cbh_.z(); }
	  Eigen::Ref<Eigen::VectorXd> bh() { return cbh_.yz(); }
	  std::vector<Eigen::Triplet<double>>& Acoeffs() { return Acoeffs_; }
	  std::vector<Eigen::Triplet<double>>& Gcoeffs() { return Gcoeffs_; }
	  const Eigen::Ref<const Eigen::VectorXd> cbh() const { return cbh_; }
	  const Eigen::Ref<const Eigen::VectorXd> c() const { return cbh_.x(); }
	  const Eigen::Ref<const Eigen::VectorXd> b() const { return cbh_.y(); }
	  const Eigen::Ref<const Eigen::VectorXd> h() const { return cbh_.z(); }
	  const Eigen::Ref<const Eigen::VectorXd> bh() const { return cbh_.yz(); }
	  const std::vector<Eigen::Triplet<double>>& Acoeffs() const { return Acoeffs_; }
	  const std::vector<Eigen::Triplet<double>>& Gcoeffs() const { return Gcoeffs_; }

	  OptimizationVector& u() { return u_opt_; }
	  OptimizationVector& v() { return v_opt_; }
	  OptimizationVector& ut() { return u_t_opt_; }
	  OptimizationVector& uprev() { return u_prev_opt_; }
	  Vector& cbh_copy() { return cbh_copy_; }
	  const OptimizationVector& u() const { return u_opt_; }
	  const OptimizationVector& v() const { return v_opt_; }
	  const OptimizationVector& ut() const { return u_t_opt_; }
	  const OptimizationVector& uprev() const { return u_prev_opt_; }
	  const Vector& cbh_copy() const { return cbh_copy_; }

    private:
	  double gTh_;
	  Cone* cone_;
	  SolverSetting* stgs_;
	  Vector cbh_, cbh_copy_;
	  Eigen::SparseMatrix<double> A_, At_, G_, Gt_;
	  std::vector<Eigen::Triplet<double>> Acoeffs_, Gcoeffs_;
	  OptimizationVector u_opt_, v_opt_, u_t_opt_, u_prev_opt_;
  };

}
