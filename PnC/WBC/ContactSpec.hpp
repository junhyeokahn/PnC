#pragma once

#include <stdio.h>
#include <Eigen/Dense>
#include <iostream>

#include <PnC/RobotSystem/RobotSystem.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

//! Contact Frame's z-axis should correspond normal vector to the ground

class ContactSpec {
 public:
  ContactSpec(RobotSystem* _robot, const int& _dim) {
    robot_ = _robot;
    dim_contact_ = _dim;
    b_set_contact_ = false;
    idx_Fz_ = dim_contact_ - 1;
    Jc_ = Eigen::MatrixXd::Zero(dim_contact_, robot_->getNumDofs());
    JcDotQdot_ = Eigen::VectorXd::Zero(dim_contact_);
  }

  virtual ~ContactSpec() {}

  void getContactJacobian(Eigen::MatrixXd& Jc) { Jc = Jc_; }
  void getJcDotQdot(Eigen::VectorXd& JcDotQdot) { JcDotQdot = JcDotQdot_; }
  int getDim() { return dim_contact_; }
  void unsetContact() { b_set_contact_ = false; }
  virtual void setMaxFz(double _max_fz) {}
  virtual double getMaxFz() { return 0.; }
  bool updateContactSpec() {
    _UpdateJc();
    _UpdateJcDotQdot();
    _UpdateUf();
    _UpdateInequalityVector();
    b_set_contact_ = true;
    return true;
  }

  virtual int getDimRFConstraint() { return Uf_.rows(); }
  const Eigen::VectorXd& getRFDesired() { return Fr_des_; }
  void setRFDesired(Eigen::VectorXd Fr_des_in) {Fr_des_ = Fr_des_in;}
  void getRFConstraintMtx(Eigen::MatrixXd& Uf) { Uf = Uf_; }
  void getRFConstraintVec(Eigen::VectorXd& ieq_vec) { ieq_vec = ieq_vec_; }

  int getFzIndex() { return idx_Fz_; }

 protected:
  virtual bool _UpdateJc() = 0;
  virtual bool _UpdateJcDotQdot() = 0;
  virtual bool _UpdateUf() = 0;
  virtual bool _UpdateInequalityVector() = 0;

  RobotSystem* robot_;
  Eigen::MatrixXd Jc_;
  Eigen::VectorXd JcDotQdot_;
  Eigen::VectorXd Fr_des_;
  int dim_contact_;
  int idx_Fz_;
  bool b_set_contact_;

  Eigen::MatrixXd Uf_;
  Eigen::VectorXd ieq_vec_;
};
