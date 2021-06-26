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
  y_vel_est_ = new AverageFilter(A1Aux::servo_rate, 0.030, 0.4);
  z_vel_est_ = new AverageFilter(A1Aux::servo_rate, 0.030, 0.2);

}

A1StateEstimator::~A1StateEstimator() {
  delete ori_est_;
  delete x_vel_est_;
  delete y_vel_est_;
  delete z_vel_est_;
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

void A1StateEstimator::Initialization(A1SensorData* data) {
  _JointUpdate(data);

  std::vector<double> trunk_acc(3);
  std::vector<double> trunk_ang_vel(3);
  MapToTorso_(data->imu_acc, data->imu_ang_vel, trunk_acc, trunk_ang_vel);

  ori_est_->estimatorInitialization(trunk_acc, trunk_ang_vel);
  ori_est_->getEstimatedState(global_body_euler_zyx_,
                              global_body_euler_zyx_dot_);

  // Apply alpha filter to angular velocity estimate
  double alphaAngularVelocity = computeAlphaGivenBreakFrequency(
      angular_velocity_filter_freq_, A1Aux::servo_rate);
  global_body_euler_zyx_dot_ =
      global_body_euler_zyx_dot_ * alphaAngularVelocity +
      (1.0 - alphaAngularVelocity) * prev_body_euler_zyx_dot_;
  prev_body_euler_zyx_dot_ = global_body_euler_zyx_dot_;

  global_body_quat_ = Eigen::Quaternion<double>(
      dart::math::eulerZYXToMatrix(global_body_euler_zyx_));

  _ConfigurationAndModelUpdate();



  sp_->jpos_ini = curr_config_.segment(A1::n_vdof, A1::n_adof);
  _FootContactUpdate(data);
  sp_->saveCurrentData();
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

void A1StateEstimator::Update(A1SensorData* data) {
  sp_->imu_ang_vel = data->imu_ang_vel;
  sp_->imu_acc = data->imu_acc;
  myUtils::pretty_print(sp_->imu_ang_vel, std::cout, "imu_ang_vel");
  myUtils::pretty_print(sp_->imu_acc, std::cout, "imu_acc");
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

  double alphaVelocity = computeAlphaGivenBreakFrequency(
      joint_velocity_filter_freq_, A1Aux::servo_rate);


  for (int i=0; i < robot_->getNumActuatedDofs(); ++i) {
    curr_config_[robot_->getNumVirtualDofs() + i] = data->q[i];
    curr_qdot_[robot_->getNumVirtualDofs() + i] =
        data->qdot[i] * alphaVelocity +
        (1.0 - alphaVelocity) * prev_qdot_[robot_->getNumVirtualDofs() + i];
  }
  prev_qdot_ = curr_qdot_;

}

void A1StateEstimator::_ConfigurationAndModelUpdate() {
  curr_config_[3] = global_body_euler_zyx_[0];
  curr_config_[4] = global_body_euler_zyx_[1];
  curr_config_[5] = global_body_euler_zyx_[2];

  for (int i(0); i < 3; ++i) curr_qdot_[i + 3] = global_body_euler_zyx_dot_[i];

  robot_->updateSystem(curr_config_, curr_qdot_, false);
  Eigen::VectorXd front_foot_pos, rear_foot_pos;
  Eigen::VectorXd front_foot_vel, rear_foot_vel;
  if(sp_->front_stance_foot == A1BodyNode::FR_foot) {
    front_foot_pos =
        robot_->getBodyNodeIsometry(A1BodyNode::FR_foot).translation();
    rear_foot_pos =
        robot_->getBodyNodeIsometry(A1BodyNode::RL_foot).translation();
    front_foot_vel =
        robot_->getBodyNodeSpatialVelocity(A1BodyNode::FR_foot).tail(3);
    rear_foot_vel =
        robot_->getBodyNodeSpatialVelocity(A1BodyNode::RL_foot).tail(3);
  } else {
    front_foot_pos =
        robot_->getBodyNodeIsometry(A1BodyNode::FL_foot).translation();
    rear_foot_pos =
        robot_->getBodyNodeIsometry(A1BodyNode::RR_foot).translation();
    front_foot_vel =
        robot_->getBodyNodeSpatialVelocity(A1BodyNode::FL_foot).tail(3);
    rear_foot_vel =
        robot_->getBodyNodeSpatialVelocity(A1BodyNode::RR_foot).tail(3);
  }

  // check if stance foot changes. If so, find the new linear offset
  if (sp_->front_stance_foot != sp_->prev_front_stance_foot) {
    Eigen::Vector3d new_front_stance_foot = front_foot_pos;
    Eigen::Vector3d new_rear_stance_foot = rear_foot_pos;
    Eigen::Vector3d old_front_stance_foot, old_rear_stance_foot;

    if(sp_->prev_front_stance_foot == A1BodyNode::FR_foot) {
      old_front_stance_foot =
          robot_->getBodyNodeIsometry(A1BodyNode::FR_foot).translation();
      old_rear_stance_foot =
          robot_->getBodyNodeIsometry(A1BodyNode::RL_foot).translation();
    } else {
      old_front_stance_foot =
          robot_->getBodyNodeIsometry(A1BodyNode::FL_foot).translation();
      old_rear_stance_foot =
          robot_->getBodyNodeIsometry(A1BodyNode::RR_foot).translation();
    }
    Eigen::Vector3d front_stance_difference = new_front_stance_foot - old_front_stance_foot;
    Eigen::Vector3d rear_stance_difference = new_rear_stance_foot - old_rear_stance_foot;

    // new and old estimates must match, so find the actual offset
    Eigen::Vector3d old_estimate = global_linear_offset_ - old_front_stance_foot;
    Eigen::Vector3d new_estimate =
            (global_linear_offset_ + front_stance_difference) - new_front_stance_foot;

    Eigen::Vector3d estimate_diff = new_estimate - old_estimate;
    Eigen::Vector3d offset_update =
            global_linear_offset_ + front_stance_difference + estimate_diff;

    global_linear_offset_ = offset_update;
    front_foot_pos = new_front_stance_foot;
  }

  // Perform Base update using kinematics
    // pushing the base upwards to put our contact feet on the ground
    // i.e contact foot position is on the ground
  curr_config_[0] = global_linear_offset_[0] - front_foot_pos[0];
  curr_config_[1] = global_linear_offset_[1] - front_foot_pos[1];
  curr_config_[2] = global_linear_offset_[2] - front_foot_pos[2];

  // Update qdot using the difference between the curr_config_ now and previous
  curr_qdot_.head(3) =
    (curr_config_.head(3) - prev_config_.head(3)) / (A1Aux::servo_rate);

  // Again call updateSystem to include linear and orientation
  robot_->updateSystem(curr_config_, curr_qdot_, false);

  sp_->q = curr_config_;
  sp_->qdot = curr_qdot_;
  sp_->prev_front_stance_foot = sp_->front_stance_foot;
  prev_config_ = curr_config_;
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


