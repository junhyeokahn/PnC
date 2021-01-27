#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/A1PnC/A1Definition.hpp>
#include <PnC/A1PnC/A1Interface.hpp>
#include <PnC/A1PnC/A1StateEstimator.hpp>
#include <PnC/A1PnC/A1StateProvider.hpp>
#include <Utils/IO/IOUtilities.hpp>

A1StateEstimator::A1StateEstimator(RobotSystem* robot) {
  myUtils::pretty_constructor(1, "A1 State Estimator");

  robot_ = robot;
  sp_ = A1StateProvider::getStateProvider(robot_);
  curr_config_ = Eigen::VectorXd::Zero(18);
  curr_qdot_ = Eigen::VectorXd::Zero(18);
}

A1StateEstimator::~A1StateEstimator() {}

void A1StateEstimator::Initialization(A1SensorData* data) {
  _JointUpdate(data);
  _ConfigurationAndModelUpdate();
  sp_->jpos_ini = curr_config_.segment(A1::n_vdof, A1::n_adof);
  _FootContactUpdate(data);
  sp_->saveCurrentData();
}

void A1StateEstimator::Update(A1SensorData* data) {
  _JointUpdate(data);
  _ConfigurationAndModelUpdate();
  _FootContactUpdate(data);
  sp_->com_vel = robot_->getCoMVelocity();
  sp_->saveCurrentData();
}

void A1StateEstimator::_JointUpdate(A1SensorData* data) {
  curr_config_.setZero();
  curr_qdot_.setZero();
  for (int i = 0; i < A1::n_vdof; ++i) {
    curr_config_[i] = data->virtual_q[i];
    curr_qdot_[i] = data->virtual_qdot[i];
  }
  for (int i(0); i < A1::n_adof; ++i) {
    curr_config_[A1::n_vdof + i] = data->q[i];
    curr_qdot_[A1::n_vdof + i] = data->qdot[i];
  }

}

void A1StateEstimator::_ConfigurationAndModelUpdate() {
  robot_->updateSystem(curr_config_, curr_qdot_, true);

  sp_->q = curr_config_;
  sp_->qdot = curr_qdot_;
}

void A1StateEstimator::_FootContactUpdate(A1SensorData* data) {
  if (data->frfoot_contact) sp_->b_frfoot_contact = 1;
  else sp_->b_frfoot_contact = 0;
  if (data->flfoot_contact) sp_->b_flfoot_contact = 1;
  else sp_->b_flfoot_contact = 0;
  if (data->rrfoot_contact) sp_->b_rrfoot_contact = 1;
  else sp_->b_rrfoot_contact = 0;
  if (data->rlfoot_contact) sp_->b_rlfoot_contact = 1;
  else sp_->b_rlfoot_contact = 0;
}


