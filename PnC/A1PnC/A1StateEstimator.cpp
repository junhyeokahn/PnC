#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/A1PnC/A1Definition.hpp>
#include <PnC/A1PnC/A1Interface.hpp>
#include <PnC/A1PnC/A1StateEstimator.hpp>
#include <PnC/A1PnC/A1StateEstimator/BasicAccumulation.hpp>
#include <PnC/A1PnC/A1StateProvider.hpp>
#include <PnC/Filter/Basic/filter.hpp>
#include <Utils/IO/IOUtilities.hpp>

A1StateEstimator::A1StateEstimator(RobotSystem* robot) {
  myUtils::pretty_constructor(1, "A1 State Estimator");

  robot_ = robot;
  sp_ = A1StateProvider::getStateProvider(robot_);

  curr_config_ = Eigen::VectorXd::Zero(robot_->getNumDofs());
  prev_config_ = Eigen::VectorXd::Zero(robot_->getNumDofs());

  global_linear_offset_.setZero();

  curr_qdot_ = Eigen::VectorXd::Zero(robot_->getNumDofs());
  prev_qdot_ = curr_qdot_;

  global_body_euler_zyx_.setZero();
  global_body_quat_ = Eigen::Quaternion<double>::Identity();

  global_body_euler_zyx_dot_.setZero();
  prev_body_euler_zyx_dot_ = global_body_euler_zyx_dot_;

  virtual_q_ = Eigen::VectorXd::Zero(6);
  virtual_qdot_ = Eigen::VectorXd::Zero(6);

  joint_velocity_filter_freq_ = 100.0;    // Hz
  angular_velocity_filter_freq_ = 100.0;  // Hz

  ori_est_ = new BasicAccumulation();
  x_vel_est_ = new AverageFilter(A1Aux::servo_rate, 0.030, 0.15);
  // y_vel_est_ = new AverageFilter(A1Aux::servo_rate, 0.030, 0.4);
  // z_vel_est_ = new AverageFilter(A1Aux::servo_rate, 0.030, 0.2);
}

A1StateEstimator::~A1StateEstimator() {
  delete ori_est_;
  delete x_vel_est_;
  // delete y_vel_est_;
  // delete z_vel_est_;
}

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
  sp_->frfoot_pos = robot_->getBodyNodeIsometry(A1BodyNode::FR_foot).translation();
  sp_->flfoot_pos = robot_->getBodyNodeIsometry(A1BodyNode::FL_foot).translation(); 
  sp_->rrfoot_pos = robot_->getBodyNodeIsometry(A1BodyNode::RR_foot).translation(); 
  sp_->rlfoot_pos = robot_->getBodyNodeIsometry(A1BodyNode::RL_foot).translation();
  sp_->frfoot_vel = robot_->getBodyNodeSpatialVelocity(A1BodyNode::FR_foot).tail(3);
  sp_->flfoot_vel = robot_->getBodyNodeSpatialVelocity(A1BodyNode::FL_foot).tail(3);
  sp_->rrfoot_vel = robot_->getBodyNodeSpatialVelocity(A1BodyNode::RR_foot).tail(3);
  sp_->rlfoot_vel = robot_->getBodyNodeSpatialVelocity(A1BodyNode::RL_foot).tail(3);
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
  sp_->com_pos = robot_->getCoMPosition();
  sp_->com_vel = robot_->getCoMVelocity();
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




double A1StateEstimator::clamp_value(double in, double min, double max) {
  if (in >= max) return max;
  else if (in <= min) return min;
  else return in;
}

double A1StateEstimator::computeAlphaGivenBreakFrequency(double hz,
                                                         double dt) {
  double omega = 2. * M_PI * hz;
  double alpha = (omega * dt) / (1. + (omega * dt));
  alpha = clamp_value(alpha, 0.0, 1.0);
  return alpha;
}



void A1StateEstimator::MapToTorso_(const Eigen::VectorXd& imu_acc,
                                   const Eigen::VectorXd& imu_angvel,
                                   std::vector<double>& trunk_acc,
                                   std::vector<double>& trunk_angvel) {

  Eigen::MatrixXd R_world_imu = Eigen::MatrixXd::Zero(3, 3);
  Eigen::MatrixXd R_world_trunk = Eigen::MatrixXd::Zero(3, 3);
  Eigen::MatrixXd R_trunk_imu = Eigen::MatrixXd::Zero(3, 3);

  Eigen::VectorXd t_acc_local = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd t_angvel_local = Eigen::VectorXd::Zero(3);

  R_world_imu = robot_->getBodyNodeIsometry(A1BodyNode::imu_link).linear();
  R_world_trunk = robot_->getBodyNodeIsometry(A1BodyNode::trunk).linear();
  R_trunk_imu = R_world_trunk.transpose() * R_world_imu;

  t_acc_local = R_trunk_imu * imu_acc;
  t_angvel_local = R_trunk_imu * imu_angvel;

  for(int i=0; i<3; ++i) {
    trunk_acc[i] = t_acc_local[i];
    trunk_angvel[i] = t_angvel_local[i];
  }
}

