#include <PnC/LaikagoPnC/LaikagoDefinition.hpp>
#include <PnC/LaikagoPnC/LaikagoInterface.hpp>
#include <PnC/LaikagoPnC/LaikagoStateEstimator.hpp>
#include <PnC/LaikagoPnC/LaikagoStateEstimator/BasicAccumulation.hpp>
#include <PnC/LaikagoPnC/LaikagoStateProvider.hpp>
#include <PnC/Filter/Basic/filter.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <Utils/IO/IOUtilities.hpp>

LaikagoStateEstimator::LaikagoStateEstimator(RobotSystem* robot) {
  myUtils::pretty_constructor(1, "Laikago State Estimator");

  robot_ = robot;
  sp_ = LaikagoStateProvider::getStateProvider(robot_);
  curr_config_ = Eigen::VectorXd::Zero(robot_->getNumDofs());

  prev_config_ = Eigen::VectorXd::Zero(robot_->getNumDofs());
  global_linear_offset_.setZero();

  curr_qdot_ = Eigen::VectorXd::Zero(robot_->getNumDofs());
  global_body_euler_zyx_.setZero();
  global_body_quat_ = Eigen::Quaternion<double>::Identity();
  global_body_euler_zyx_dot_.setZero();
  virtual_q_ = Eigen::VectorXd::Zero(6);
  virtual_qdot_ = Eigen::VectorXd::Zero(6);

  prev_qdot_ = curr_qdot_;
  prev_body_euler_zyx_dot_ = global_body_euler_zyx_dot_;

  joint_velocity_filter_freq_ = 100.0;    // Hz
  angular_velocity_filter_freq_ = 100.0;  // Hz

  ori_est_ = new BasicAccumulation();
  // x_vel_est_ = new AverageFilter(LaikagoAux::servo_rate, 0.015, 1.0);
  // y_vel_est_ = new AverageFilter(LaikagoAux::servo_rate, 0.015, 1.5);
  // z_vel_est_ = new AverageFilter(LaikagoAux::servo_rate, 0.015, 1.5);
  // TODO
  x_vel_est_ = new AverageFilter(LaikagoAux::servo_rate, 0.030, 0.15);
  y_vel_est_ = new AverageFilter(LaikagoAux::servo_rate, 0.030, 0.4);
  z_vel_est_ = new AverageFilter(LaikagoAux::servo_rate, 0.030, 0.2);
}

LaikagoStateEstimator::~LaikagoStateEstimator() {
  delete ori_est_;
  delete x_vel_est_;
  delete y_vel_est_;
  delete z_vel_est_;
}

double LaikagoStateEstimator::clamp_value(double in, double min, double max) {
  if (in >= max) {
    return max;
  } else if (in <= min) {
    return min;
  } else {
    return in;
  }
}

double LaikagoStateEstimator::computeAlphaGivenBreakFrequency(double hz,
                                                            double dt) {
  double omega = 2.0 * M_PI * hz;
  // double alpha = (omega * dt / 2.0) / (1.0 + (omega * dt / 2.0));
  double alpha = (omega * dt) / (1.0 + (omega * dt));
  alpha = clamp_value(alpha, 0.0, 1.0);
  return alpha;
}

void LaikagoStateEstimator::_UpdateDCM() {
  sp_->com_pos = robot_->getCoMPosition();
  sp_->com_vel = robot_->getCoMVelocity();
  sp_->dcm_omega = sqrt(9.81 / sp_->com_pos[2]);
  sp_->prev_dcm = sp_->dcm;
  sp_->dcm = robot_->getCoMPosition() + sp_->est_com_vel / sp_->dcm_omega;
  double alpha_vel = 0.1;
  sp_->dcm_vel =
      alpha_vel * ((sp_->dcm - sp_->prev_dcm) / LaikagoAux::servo_rate) +
      (1.0 - alpha_vel) * sp_->dcm_vel;
  sp_->r_vrp = sp_->dcm - (sp_->dcm_vel / sp_->dcm_omega);
}

void LaikagoStateEstimator::initialization(LaikagoSensorData* data) {
  _JointUpdate(data);

  std::vector<double> torso_acc(3);
  std::vector<double> torso_ang_vel(3);
  MapToTorso_(data->imu_acc, data->imu_ang_vel, torso_acc, torso_ang_vel);

  ori_est_->estimatorInitialization(torso_acc, torso_ang_vel);
  ori_est_->getEstimatedState(global_body_euler_zyx_,
                              global_body_euler_zyx_dot_);

  // Apply alpha filter to angular velocity estimate
  double alphaAngularVelocity = computeAlphaGivenBreakFrequency(
      angular_velocity_filter_freq_, LaikagoAux::servo_rate);
  global_body_euler_zyx_dot_ =
      global_body_euler_zyx_dot_ * alphaAngularVelocity +
      (1.0 - alphaAngularVelocity) * prev_body_euler_zyx_dot_;
  prev_body_euler_zyx_dot_ = global_body_euler_zyx_dot_;

  global_body_quat_ = Eigen::Quaternion<double>(
      dart::math::eulerZYXToMatrix(global_body_euler_zyx_));

  _ConfigurationAndModelUpdate();
  _UpdateDCM();

  ((AverageFilter*)x_vel_est_)->initialization(sp_->com_vel[0]);
  ((AverageFilter*)y_vel_est_)->initialization(sp_->com_vel[1]);
  ((AverageFilter*)z_vel_est_)->initialization(sp_->com_vel[2]);

  _FootContactUpdate(data);

  sp_->saveCurrentData();
}

void LaikagoStateEstimator::MapToTorso_(const Eigen::VectorXd& imu_acc,
                                      const Eigen::VectorXd& imu_angvel,
                                      std::vector<double>& torso_acc,
                                      std::vector<double>& torso_angvel) {
  Eigen::MatrixXd R_world_imu = Eigen::MatrixXd::Zero(3, 3);
  Eigen::MatrixXd R_world_torso = Eigen::MatrixXd::Zero(3, 3);
  Eigen::MatrixXd R_torso_imu = Eigen::MatrixXd::Zero(3, 3);

  Eigen::VectorXd t_acc_local = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd t_angvel_local = Eigen::VectorXd::Zero(3);

  R_world_imu = robot_->getBodyNodeIsometry(LaikagoBodyNode::IMU).linear();
  R_world_torso = robot_->getBodyNodeIsometry(LaikagoBodyNode::trunk).linear();
  R_torso_imu = R_world_torso.transpose() * R_world_imu;

  t_acc_local = R_torso_imu * imu_acc;
  t_angvel_local = R_torso_imu * imu_angvel;

  for (int i = 0; i < 3; ++i) {
    torso_acc[i] = t_acc_local[i];
    torso_angvel[i] = t_angvel_local[i];
  }
}

void LaikagoStateEstimator::update(LaikagoSensorData* data) {
  _JointUpdate(data);

  std::vector<double> torso_acc(3);
  std::vector<double> torso_ang_vel(3);

  MapToTorso_(data->imu_acc, data->imu_ang_vel, torso_acc, torso_ang_vel);

  ori_est_->setSensorData(torso_acc, torso_ang_vel);
  ori_est_->getEstimatedState(global_body_euler_zyx_,
                              global_body_euler_zyx_dot_);

  // Apply alpha filter to angular velocity estimate
  double alphaAngularVelocity = computeAlphaGivenBreakFrequency(
      angular_velocity_filter_freq_, LaikagoAux::servo_rate);
  global_body_euler_zyx_dot_ =
      global_body_euler_zyx_dot_ * alphaAngularVelocity +
      (1.0 - alphaAngularVelocity) * prev_body_euler_zyx_dot_;
  prev_body_euler_zyx_dot_ = global_body_euler_zyx_dot_;

  global_body_quat_ = Eigen::Quaternion<double>(
      dart::math::eulerZYXToMatrix(global_body_euler_zyx_));

  _ConfigurationAndModelUpdate();
  _UpdateDCM();

  static bool visit_once(false);
  if ((sp_->phase_copy == 2) && (!visit_once)) {
    ((AverageFilter*)x_vel_est_)->initialization(sp_->com_vel[0]);
    ((AverageFilter*)y_vel_est_)->initialization(sp_->com_vel[1]);
    ((AverageFilter*)z_vel_est_)->initialization(sp_->com_vel[2]);
    visit_once = true;
  }

  x_vel_est_->input(sp_->com_vel[0]);
  sp_->est_com_vel[0] = x_vel_est_->output();
  y_vel_est_->input(sp_->com_vel[1]);
  sp_->est_com_vel[1] = y_vel_est_->output();
  z_vel_est_->input(sp_->com_vel[2]);
  sp_->est_com_vel[2] = z_vel_est_->output();

  _FootContactUpdate(data);

  sp_->saveCurrentData();
}

void LaikagoStateEstimator::_JointUpdate(LaikagoSensorData* data) {
  curr_config_.setZero();
  curr_qdot_.setZero();

  double alphaVelocity = computeAlphaGivenBreakFrequency(
      joint_velocity_filter_freq_, LaikagoAux::servo_rate);

  for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
    curr_config_[robot_->getNumVirtualDofs() + i] = data->q[i];
    // curr_qdot_[robot_->getNumVirtualDofs() + i] = data->qdot[i];
    // Apply alpha filter on joint velocities
    curr_qdot_[robot_->getNumVirtualDofs() + i] =
        data->qdot[i] * alphaVelocity +
        (1.0 - alphaVelocity) * prev_qdot_[robot_->getNumVirtualDofs() + i];
  }
  prev_qdot_ = curr_qdot_;
  sp_->l_rf = data->lf_wrench;
  sp_->r_rf = data->rf_wrench;
  sp_->rotor_inertia = data->rotor_inertia;
}

void LaikagoStateEstimator::_ConfigurationAndModelUpdate() {
  curr_config_[3] = global_body_euler_zyx_[0];
  curr_config_[4] = global_body_euler_zyx_[1];
  curr_config_[5] = global_body_euler_zyx_[2];

  for (int i(0); i < 3; ++i) curr_qdot_[i + 3] = global_body_euler_zyx_dot_[i];

  robot_->updateSystem(curr_config_, curr_qdot_, false);
  Eigen::VectorXd foot_pos;
  Eigen::VectorXd foot_vel;
  if (sp_->stance_foot == LaikagoBodyNode::FR_foot) {
    foot_pos =
        robot_->getBodyNodeIsometry(LaikagoBodyNode::FR_foot).translation();
    foot_vel =
        robot_->getBodyNodeSpatialVelocity(LaikagoBodyNode::FR_foot).tail(3);
  } else {
    foot_pos =
        robot_->getBodyNodeIsometry(LaikagoBodyNode::FL_foot).translation();
    foot_vel =
        robot_->getBodyNodeSpatialVelocity(LaikagoBodyNode::FL_foot).tail(3);
  }

  // check if stance foot changes. If so, find the new linear offset
  if (sp_->stance_foot != sp_->prev_stance_foot) {
    Eigen::Vector3d new_stance_foot = foot_pos;
    Eigen::Vector3d old_stance_foot;
    if (sp_->prev_stance_foot == LaikagoBodyNode::FR_foot) {
      old_stance_foot =
          robot_->getBodyNodeIsometry(LaikagoBodyNode::FR_foot).translation();
    } else {
      old_stance_foot =
          robot_->getBodyNodeIsometry(LaikagoBodyNode::FL_foot).translation();
    }
    Eigen::Vector3d stance_difference = new_stance_foot - old_stance_foot;

    // new and old estimates must match, so find the actual offset
    Eigen::Vector3d old_estimate = global_linear_offset_ - old_stance_foot;
    Eigen::Vector3d new_estimate =
        (global_linear_offset_ + stance_difference) - new_stance_foot;
    Eigen::Vector3d estimate_diff = new_estimate - old_estimate;

    Eigen::Vector3d offset_update =
        global_linear_offset_ + stance_difference + estimate_diff;

    // myUtils::pretty_print(new_stance_foot, std::cout, "new_stance_foot");
    // myUtils::pretty_print(old_stance_foot, std::cout, "old_stance_foot");
    // myUtils::pretty_print(stance_difference, std::cout, "stance_difference");
    // myUtils::pretty_print(old_estimate, std::cout, "old_estimate");
    // myUtils::pretty_print(new_estimate, std::cout, "new_estimate");
    // myUtils::pretty_print(offset_update, std::cout, "offset_update");

    // Update foot positions
    global_linear_offset_ = offset_update;
    foot_pos = new_stance_foot;

    // Eigen::Vector3d actual_config = global_linear_offset_ - new_stance_foot;
    // myUtils::pretty_print(actual_config, std::cout, "actual_config");
  }

  // Perform Base update using kinematics
  curr_config_[0] = global_linear_offset_[0] - foot_pos[0];
  curr_config_[1] = global_linear_offset_[1] - foot_pos[1];
  curr_config_[2] = global_linear_offset_[2] - foot_pos[2];

  // Update qdot using the difference between the curr_config_ now and previous
  curr_qdot_.head(3) =
      (curr_config_.head(3) - prev_config_.head(3)) / (LaikagoAux::servo_rate);

  // curr_qdot_[0] = -foot_vel[0];
  // curr_qdot_[1] = -foot_vel[1];
  // curr_qdot_[2] = -foot_vel[2];

  // Eigen::Vector3d config_xyz = curr_config_.head(3);
  // Eigen::Vector3d config_dot_xyz = curr_qdot_.head(3);
  // myUtils::pretty_print(config_xyz, std::cout, "config_xyz");
  // Eigen::Vector3d config_dot_xyz_estimate = (curr_config_.head(3) -
  // prev_config_.head(3))/LaikagoAux::servo_rate;
  // myUtils::pretty_print(config_dot_xyz, std::cout, "config_dot_xyz");
  // myUtils::pretty_print(config_dot_xyz_estimate, std::cout,
  // "config_dot_xyz_estimate");

  robot_->updateSystem(curr_config_, curr_qdot_, false);

  sp_->q = curr_config_;
  sp_->qdot = curr_qdot_;
  // update previous stance foot.
  sp_->prev_stance_foot = sp_->stance_foot;
  // update previous config:
  prev_config_ = curr_config_;
}

void LaikagoStateEstimator::_FootContactUpdate(LaikagoSensorData* data) {
  if (data->rfoot_contact)
    sp_->b_rfoot_contact = 1;
  else
    sp_->b_rfoot_contact = 0;
  if (data->lfoot_contact)
    sp_->b_lfoot_contact = 1;
  else
    sp_->b_lfoot_contact = 0;
}
