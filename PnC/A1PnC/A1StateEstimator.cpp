#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/A1PnC/A1Definition.hpp>
#include <PnC/A1PnC/A1Interface.hpp>
#include <PnC/A1PnC/A1StateEstimator.hpp>
#include <PnC/A1PnC/A1StateEstimator/BasicAccumulation.hpp>
#include <PnC/A1PnC/A1StateProvider.hpp>
// #include <PnC/Filter/Basic/filter.hpp>
#include <Utils/IO/IOUtilities.hpp>




AverageFilter::AverageFilter(double dt, double t_const, double limit)
      : dt_(dt), t_const_(t_const), limit_(limit) {
  myUtils::pretty_constructor(2, "Average Filter");
  est_value_ = 0.;
}

AverageFilter::~AverageFilter() {}

void AverageFilter::initialization(double _val) { est_value_ = _val; }

void AverageFilter::clear() { est_value_ = 0.; }

void AverageFilter::input(double input) {
  double update_value = input - est_value_;
  if (fabs(update_value) > limit_) {
    update_value = 0.;
  }
  est_value_ += (dt_ / (dt_ + t_const_)) * update_value;
}

double AverageFilter::output() { return est_value_; }


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

  // TODO: Find t_const and limit vals
  x_vel_est_ = new AverageFilter(A1Aux::servo_rate, 0.030, 0.15);
  y_vel_est_ = new AverageFilter(A1Aux::servo_rate, 0.030, 0.4);
  z_vel_est_ = new AverageFilter(A1Aux::servo_rate, 0.030, 0.2);
}

A1StateEstimator::~A1StateEstimator() {
  delete x_vel_est_;
  delete y_vel_est_;
  delete z_vel_est_;
}

void A1StateEstimator::Initialization(A1SensorData* data) {
  _COMAngularUpdate(data);
  _JointUpdate(data);
  _ConfigurationAndModelUpdate(data);
  sp_->jpos_ini = curr_config_.segment(A1::n_vdof, A1::n_adof);

  x_vel_est_->initialization(sp_->com_vel[0]);
  y_vel_est_->initialization(sp_->com_vel[1]);
  z_vel_est_->initialization(sp_->com_vel[2]);

  _FootContactUpdate(data);
  sp_->saveCurrentData();
}

void A1StateEstimator::_COMAngularUpdate(A1SensorData* data) {
    // TODO: 1) NED->Robot World Frame
    //       2) imu_link->trunk
  global_body_euler_zyx_[0] = data->imu_rpy[2];
  global_body_euler_zyx_[1] = data->imu_rpy[1];
  global_body_euler_zyx_[2] = data->imu_rpy[0];

  global_body_euler_zyx_dot_[0] = data->imu_ang_vel[2];
  global_body_euler_zyx_dot_[1] = data->imu_ang_vel[1];
  global_body_euler_zyx_dot_[2] = data->imu_ang_vel[0];

  global_body_quat_ = Eigen::Quaternion<double> (
        dart::math::eulerZYXToMatrix(global_body_euler_zyx_));
}

void A1StateEstimator::Update(A1SensorData* data) {
  // TODO: NED->RobotWorld then IMU->trunk
  _COMAngularUpdate(data);

  _JointUpdate(data);

  std::vector<double> trunk_acc(3), trunk_ang_vel(3);

  _ConfigurationAndModelUpdate(data);

  // TODO: Below Block Taken from DraceStateEstimator, what is phase_copy == 2?
  // static bool visit_once(false);
  // if ((sp_->phase_copy == 2) && (!visit_once)) {
  //   ((AverageFilter*)x_vel_est_)->initialization(sp_->com_vel[0]);
  //   ((AverageFilter*)y_vel_est_)->initialization(sp_->com_vel[1]);
  //   ((AverageFilter*)z_vel_est_)->initialization(sp_->com_vel[2]);
  //   visit_once = true;
  // }

  x_vel_est_->input(sp_->com_vel[0]);
  sp_->est_com_vel[0] = x_vel_est_->output();

  y_vel_est_->input(sp_->com_vel[1]);
  sp_->est_com_vel[1] = y_vel_est_->output();

  z_vel_est_->input(sp_->com_vel[2]);
  sp_->est_com_vel[2] = 1_vel_est_->output();

  _FootContactUpdate(data);

  sp_->frfoot_pos = robot_->getBodyNodeIsometry(A1BodyNode::FR_foot).translation();
  sp_->flfoot_pos = robot_->getBodyNodeIsometry(A1BodyNode::FL_foot).translation(); 
  sp_->rrfoot_pos = robot_->getBodyNodeIsometry(A1BodyNode::RR_foot).translation(); 
  sp_->rlfoot_pos = robot_->getBodyNodeIsometry(A1BodyNode::RL_foot).translation();
  sp_->frfoot_vel = robot_->getBodyNodeSpatialVelocity(A1BodyNode::FR_foot).tail(3);
  sp_->flfoot_vel = robot_->getBodyNodeSpatialVelocity(A1BodyNode::FL_foot).tail(3);
  sp_->rrfoot_vel = robot_->getBodyNodeSpatialVelocity(A1BodyNode::RR_foot).tail(3);
  sp_->rlfoot_vel = robot_->getBodyNodeSpatialVelocity(A1BodyNode::RL_foot).tail(3);
  sp_->imu_ang_vel = data->imu_ang_vel;
  sp_->imu_acc = data->imu_acc;
  sp_->imu_rpy = data->imu_rpy;

  sp_->saveCurrentData();
}

void A1StateEstimator::_JointUpdate(A1SensorData* data) {
  curr_config_.setZero();
  curr_qdot_.setZero();
  // Removed the code block below because we only have in sim
  /*for (int i = 0; i < A1::n_vdof; ++i) {
    curr_config_[i] = data->virtual_q[i];
    curr_qdot_[i] = data->virtual_qdot[i];
  }*/
  for (int i(0); i < A1::n_adof; ++i) {
    curr_config_[A1::n_vdof + i] = data->q[i];
    curr_qdot_[A1::n_vdof + i] = data->qdot[i];
  }

}

void A1StateEstimator::_ConfigurationAndModelUpdate(A1SensorData* data) {
  robot_->updateSystem(curr_config_, curr_qdot_, true);

    for (int i(0); i < 3; ++i) curr_qdot_[i + 3] = global_body_euler_zyx_dot_[i];

  robot_->updateSystem(curr_config_, curr_qdot_, false); // update robot as is correct ori but 0,0,0 floating base
  Eigen::VectorXd foot_pos;
  Eigen::VectorXd foot_vel;
  if (sp_->front_stance_foot == A1BodyNode::FR_foot) {
    foot_pos =
        robot_->getBodyNodeIsometry(A1BodyNode::FR_foot).translation();
    foot_vel =  sp_->q = curr_config_;
  sp_->qdot = curr_qdot_;

        robot_->getBodyNodeSpatialVelocity(A1BodyNode::FR_foot).tail(3);
  } else {
    foot_pos =
        robot_->getBodyNodeIsometry(A1BodyNode::FL_foot).translation();
    foot_vel =
        robot_->getBodyNodeSpatialVelocity(A1BodyNode::FL_foot).tail(3);
  }// Foot pos will be under the ground

  // check if stance foot changes. If so, find the new linear offset
  if (sp_->front_stance_foot != sp_->prev_front_stance_foot) {
    Eigen::Vector3d new_stance_foot = foot_pos;
    Eigen::Vector3d old_stance_foot;
    if (sp_->prev_front_stance_foot == A1BodyNode::FR_foot) {
      old_stance_foot =
          robot_->getBodyNodeIsometry(A1BodyNode::FR_foot).translation();
    } else {
      old_stance_foot =
          robot_->getBodyNodeIsometry(A1BodyNode::FL_foot).translation();
    }
    Eigen::Vector3d stance_difference = new_stance_foot - old_stance_foot;

    // new and old estimates must match, so find the actual offset
    Eigen::Vector3d old_to_new = new_stance_foot - old_stance_foot;
    global_linear_offset_ += old_to_new;
    // myUtils::pretty_print(new_stance_foot, std::cout, "new_stance_foot");
    // myUtils::pretty_print(old_stance_foot, std::cout, "old_stance_foot");
    // myUtils::pretty_print(stance_difference, std::cout, "stance_difference");
    // myUtils::pretty_print(old_estimate, std::cout, "old_estimate");
    // myUtils::pretty_print(new_estimate, std::cout, "new_estimate");
    // myUtils::pretty_print(offset_update, std::cout, "offset_update");

    foot_pos = new_stance_foot;
  }
  // TODO: See June version.. some rotation pieces missing
  // Perform Base update using kinematics
  curr_config_[0] = global_linear_offset_[0] - foot_pos[0];// pushing the base upwards to put our contact feet on the ground
  curr_config_[1] = global_linear_offset_[1] - foot_pos[1];// i.e contact foot position is on ground
  curr_config_[2] = global_linear_offset_[2] - foot_pos[2];

  // Update qdot using the difference between the curr_config_ now and previous
  curr_qdot_.head(3) =
      (curr_config_.head(3) - prev_config_.head(3)) / (A1Aux::servo_rate);

  robot_->updateSystem(curr_config_, curr_qdot_, false);// Again call updateSystem to include linear and orientation 
  sp_->q = curr_config_;
  sp_->qdot = curr_qdot_;
  sp_->com_pos = robot_->getCoMPosition();
  sp_->com_vel = robot_->getCoMVelocity();

  // TODO
  curr_config_[3] = global_body_euler_zyx_[0];
  curr_config_[4] = global_body_euler_zyx_[1];
  curr_config_[5] = global_body_euler_zyx_[2];

  // update previous stance foot.
  sp_->prev_front_stance_foot = sp_->front_stance_foot;
  // update previous config:
  prev_config_ = curr_config_;
}

void A1StateEstimator::_FootContactUpdate(A1SensorData* data) {
  sp_->foot_force = data->foot_force;

  // TODO: New contact method
  if (data->foot_force[0] >= -5) sp_->b_frfoot_contact = 1;
  else sp_->b_frfoot_contact = 0;
  if (data->foot_force[1] >= -5) sp_->b_flfoot_contact = 1;
  else sp_->b_flfoot_contact = 0;
  if (data->foot_force[0] >= -1) sp_->b_rrfoot_contact = 1;
  else sp_->b_rrfoot_contact = 0;
  if (data->foot_force[0] >= -1) sp_->b_rlfoot_contact = 1;
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


// TODO needs to be added, but I dont think we need it all?
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


