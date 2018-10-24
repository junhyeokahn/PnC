#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>
#include <Utils/Utilities.hpp>

FixedBodyContact::FixedBodyContact(RobotSystem* _robot) : ContactSpec(_robot, 6)
{
  robot_ = _robot;
  Jc_ = Eigen::MatrixXd::Zero(dim_contact_, robot_->getNumDofs());

  //printf("[Fixed Body Contact] Constructed\n");
}

FixedBodyContact::~FixedBodyContact(){ }

bool FixedBodyContact::_UpdateJc(){
  for(int i(0);i<dim_contact_; ++i)  Jc_(i,i) = 1.;
  return true;
}

bool FixedBodyContact::_UpdateJcDotQdot(){
  JcDotQdot_ = Eigen::VectorXd::Zero(dim_contact_);
  return true;
}

bool FixedBodyContact::_UpdateUf(){
  Uf_ = Eigen::MatrixXd::Zero(1, dim_contact_);
  return true;
}

bool FixedBodyContact::_UpdateInequalityVector(){
  ieq_vec_ = Eigen::VectorXd::Zero(1);
  return true;
}
