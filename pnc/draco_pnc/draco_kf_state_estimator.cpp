#include "draco_kf_state_estimator.hpp"

DracoKFStateEstimator::DracoKFStateEstimator(RobotSystem *_robot) {
  util::PrettyConstructor(1, "DracoKFStateEstimator");
  robot_ = _robot;
  sp_ = DracoStateProvider::getStateProvider();

  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");

  iso_imu_to_base_com_ = robot_->get_link_iso("torso_imu").inverse() *
                         robot_->get_link_iso("torso_link");

  Eigen::Vector3d sigma_base_vel = util::ReadParameter<Eigen::Vector3d>(
          cfg["state_estimator"], "sigma_base_vel");
  Eigen::Vector3d sigma_base_acc = util::ReadParameter<Eigen::Vector3d>(
          cfg["state_estimator"], "sigma_base_acc");
  Eigen::Vector3d sigma_pos_lfoot = util::ReadParameter<Eigen::Vector3d>(
          cfg["state_estimator"], "sigma_pos_lfoot");
  Eigen::Vector3d sigma_pos_rfoot = util::ReadParameter<Eigen::Vector3d>(
          cfg["state_estimator"], "sigma_pos_rfoot");
  Eigen::Vector3d sigma_vel_lfoot = util::ReadParameter<Eigen::Vector3d>(
          cfg["state_estimator"], "sigma_vel_lfoot");
  Eigen::Vector3d sigma_vel_rfoot = util::ReadParameter<Eigen::Vector3d>(
          cfg["state_estimator"], "sigma_vel_rfoot");

  base_acceleration_.setZero();
  x_hat_.setZero();
  system_model_.initialize(deltat, sigma_base_vel, sigma_base_acc, sigma_vel_lfoot, sigma_vel_rfoot);
  base_pose_model_.initialize(sigma_pos_lfoot, sigma_pos_rfoot);
  rot_world_to_base.setZero();
  global_linear_offset_.setZero();
  prev_base_com_pos_.setZero();
  current_support_state_ = DOUBLE;
  prev_support_state_ = DOUBLE;
  foot_pos_from_base_pre_transition.setZero();
  foot_pos_from_base_post_transition(0) = NAN;

  Eigen::VectorXd n_data_com_vel = util::ReadParameter<Eigen::VectorXd>(
      cfg["state_estimator"], "n_data_com_vel");
  Eigen::VectorXd n_data_cam = util::ReadParameter<Eigen::VectorXd>(
      cfg["state_estimator"], "n_data_cam");
  Eigen::VectorXd n_data_base_accel = util::ReadParameter<Eigen::VectorXd>(
      cfg["state_estimator"], "n_data_base_accel");
  Eigen::VectorXd n_data_ang_vel = util::ReadParameter<Eigen::VectorXd>(
      cfg["state_estimator"], "n_data_ang_vel");

  for (int i = 0; i < 3; ++i) {
    com_vel_filter_.push_back(SimpleMovingAverage(n_data_com_vel[i]));
    cam_filter_.push_back(SimpleMovingAverage(n_data_cam[i]));
    base_accel_filter_.push_back(SimpleMovingAverage(n_data_base_accel[i]));
    imu_ang_vel_filter_.push_back(SimpleMovingAverage(n_data_ang_vel[i]));
  }

  // TODO move settings to config/draco/pnc.yaml
  b_first_visit_ = true;
  b_skip_prediction = false;
  b_use_marg_filter = false;
  b_request_offset_reset = false;
}

DracoKFStateEstimator::~DracoKFStateEstimator() {}

void DracoKFStateEstimator::initialize(DracoSensorData *data) {
  this->update(data);
}

void DracoKFStateEstimator::update(DracoSensorData *data) {

  // filter imu angular velocity
  for (int i = 0; i < 3; ++i) {
    imu_ang_vel_filter_[i].Input(data->imu_frame_vel[i]);
    sp_->imu_ang_vel_est[i] = imu_ang_vel_filter_[i].Output();
  }

  // estimate 0_R_b
  Eigen::Matrix3d rot_world_to_imu = data->imu_frame_iso.block(0, 0, 3, 3);
  rot_world_to_base =
      compute_world_to_base_rot(data, rot_world_to_imu, b_use_marg_filter);

  // compute estimator (control) input, u_n
  for (int i = 0; i < 3; ++i) {
    base_accel_filter_[i].Input(data->imu_dvel[i] / sp_->servo_dt);
    base_acceleration_[i] = base_accel_filter_[i].Output();
  }
  base_pose_model_.packAccelerationInput(rot_world_to_imu, base_acceleration_,
                                         accelerometer_input_);

  // update system without base linear states
  robot_->update_system(
      Eigen::Vector3d::Zero(), Eigen::Quaternion<double>(rot_world_to_base),
      Eigen::Vector3d::Zero(), data->imu_frame_vel.head(3),
      Eigen::Vector3d::Zero(), Eigen::Quaternion<double>(rot_world_to_base),
      Eigen::Vector3d::Zero(), data->imu_frame_vel.head(3),
      data->joint_positions, data->joint_velocities, false);

  // continue only when state estimator is on
  if (!(sp_->isStateEstimatorOn()))
    return;

  // get initial base estimate
  Eigen::Vector3d world_to_base;
  // initialize Kalman filter state xhat =[0_pos_b, 0_vel_b, 0_pos_LF, 0_pos_RF]
  if (b_first_visit_) {
    world_to_base = global_linear_offset_ -
                    robot_->get_link_iso("l_foot_contact").translation();

    x_hat_.initialize(world_to_base, robot_->get_link_iso("l_foot_contact"),
                      robot_->get_link_iso("r_foot_contact"));
    kalman_filter_.init(x_hat_);
  } else {
    world_to_base << x_hat_.base_pos_x(), x_hat_.base_pos_y(),
        x_hat_.base_pos_z();
  }

  // update contact
  //  if (data->b_rf_contact) {
  //    sp_->b_rf_contact = true;
  //  } else {
  //    sp_->b_rf_contact = false;
  //  }
  //
  //  if (data->b_lf_contact) {
  //    sp_->b_lf_contact = true;
  //  } else {
  //    sp_->b_lf_contact = false;
  //  }
  updateSupportState(sp_, current_support_state_);

  // at support state change, update global offset and covariance gains
  if (current_support_state_ != prev_support_state_) {

    // from double to right support and viceversa
    if ((prev_support_state_ == DOUBLE) && (current_support_state_ == RIGHT)) {
      foot_pos_from_base_pre_transition =
          x_hat_.tail(6).head(3); // estimated lfoot before lift-off
      base_pose_model_.update_leg_covariance(PoseMeasurementModel::LEFT,
                                             COV_LEVEL_HIGH);
    } else if ((prev_support_state_ == RIGHT) &&
               (current_support_state_ == DOUBLE)) {
      foot_pos_from_base_post_transition =
          x_hat_.head(3) + robot_->get_link_iso("l_foot_contact").translation();
      global_linear_offset_ = foot_pos_from_base_post_transition -
                              foot_pos_from_base_pre_transition;
      global_linear_offset_.z() =
          0.0; // TODO make more robust later for non-flat ground
      system_model_.update_lfoot_offset(global_linear_offset_);
      b_request_offset_reset = true;
      foot_pos_from_base_post_transition.z() =
          0.0; // TODO make more robust later for non-flat ground
      //      system_model_.update_lfoot_offset(foot_pos_from_base_post_transition);
      //      base_pose_model_.update_lfoot_offset(foot_pos_from_base_post_transition);
      base_pose_model_.update_leg_covariance(PoseMeasurementModel::LEFT,
                                             COV_LEVEL_LOW);
    } else if ((prev_support_state_ == DOUBLE) &&
               (current_support_state_ == LEFT)) {
      // from double support to left support and viceversa
      foot_pos_from_base_pre_transition =
          x_hat_.tail(3); // estimated rfoot before lift-off
      base_pose_model_.update_leg_covariance(PoseMeasurementModel::RIGHT,
                                             COV_LEVEL_HIGH);
    } else if ((prev_support_state_ == LEFT) &&
               (current_support_state_ == DOUBLE)) {
      foot_pos_from_base_post_transition =
          x_hat_.head(3) + robot_->get_link_iso("r_foot_contact").translation();
      global_linear_offset_ = foot_pos_from_base_post_transition -
                              foot_pos_from_base_pre_transition;
      global_linear_offset_.z() =
          0.0; // TODO make more robust later for non-flat ground
      system_model_.update_rfoot_offset(global_linear_offset_);
      b_request_offset_reset = true;
      foot_pos_from_base_post_transition.z() =
          0.0; // TODO make more robust later for non-flat ground
      //      system_model_.update_rfoot_offset(foot_pos_from_base_post_transition);
      base_pose_model_.update_leg_covariance(PoseMeasurementModel::RIGHT,
                                             COV_LEVEL_LOW);
    }
    if (foot_pos_from_base_post_transition.hasNaN()) {
      x_hat_ = kalman_filter_.predict(system_model_, accelerometer_input_);
      global_linear_offset_.setZero();
      system_model_.update_lfoot_offset(global_linear_offset_);
      system_model_.update_rfoot_offset(global_linear_offset_);
      b_skip_prediction = true;
    }
  }
  if (!b_skip_prediction) {
    x_hat_ = kalman_filter_.predict(system_model_, accelerometer_input_);
    if (b_request_offset_reset) {
      system_model_.reset_offsets();
      b_request_offset_reset = false;
    }
  } else {
    b_skip_prediction = false;
  }

  // update measurement assuming at least one foot is on the ground
  if (sp_->b_lf_contact) {
    Eigen::Vector3d pos_base_from_lfoot =
        -robot_->get_link_iso("l_foot_contact").translation();
    base_pose_model_.update_position_from_lfoot(pos_base_from_lfoot,
                                                base_estimate_);
  }
  if (sp_->b_rf_contact) {
    Eigen::Vector3d pos_base_from_rfoot =
        -robot_->get_link_iso("r_foot_contact").translation();
    base_pose_model_.update_position_from_rfoot(pos_base_from_rfoot,
                                                base_estimate_);
  }
  x_hat_ = kalman_filter_.update(base_pose_model_, base_estimate_);

  // values computed by linear KF estimator
  Eigen::Vector3d base_position_estimate(
      x_hat_.base_pos_x(), x_hat_.base_pos_y(), x_hat_.base_pos_z());
  Eigen::Vector3d base_velocity_estimate(
      x_hat_.base_vel_x(), x_hat_.base_vel_y(), x_hat_.base_vel_z());

  Eigen::Vector3d base_com_pos =
      base_position_estimate +
      rot_world_to_base * robot_->get_base_local_com_pos();
  if (b_first_visit_) {
    prev_base_com_pos_ = base_com_pos;
    b_first_visit_ = false;
  }

  Eigen::Vector3d base_com_lin_vel =
      (base_com_pos - prev_base_com_pos_) / sp_->servo_dt;

  robot_->update_system(
      //          base_com_pos, margFilter_.getQuaternion(),
      base_com_pos, Eigen::Quaternion<double>(rot_world_to_base),
      base_com_lin_vel, data->imu_frame_vel.head(3), base_position_estimate,
      //          margFilter_.getQuaternion(), base_velocity_estimate,
      Eigen::Quaternion<double>(rot_world_to_base), base_velocity_estimate,
      data->imu_frame_vel.head(3), data->joint_positions,
      data->joint_velocities, true);

  // filter com velocity, cam
  for (int i = 0; i < 3; ++i) {
    com_vel_filter_[i].Input(robot_->get_com_lin_vel()[i]);
    sp_->com_vel_est[i] = com_vel_filter_[i].Output();
    cam_filter_[i].Input(robot_->hg[i]);
    sp_->cam_est[i] = cam_filter_[i].Output();
  }

  this->ComputeDCM();

  prev_support_state_ = current_support_state_;
  // save current time step data
  if (sp_->count % sp_->save_freq == 0) {
    DracoDataManager *dm = DracoDataManager::GetDracoDataManager();
    dm->data->joint_positions = robot_->get_q().tail(robot_->n_a);
    dm->data->joint_velocities = robot_->get_q_dot().tail(robot_->n_a);
    dm->data->base_pos_kf = base_position_estimate;
    dm->data->base_vel_kf = base_velocity_estimate;
    dm->data->base_euler_kf =
        util::QuatToEulerZYX(Eigen::Quaterniond(rot_world_to_base));
    Eigen::Quaternion<double> quat =
        Eigen::Quaternion<double>(rot_world_to_base);
    dm->data->base_quat_kf =
        Eigen::Matrix<double, 4, 1>(quat.w(), quat.x(), quat.y(), quat.z());
    //    dm->data->base_quat_kf =
    //    Eigen::Vector4d(margFilter_.getQuaternion().w(),
    //                                             margFilter_.getQuaternion().x(),
    //                                             margFilter_.getQuaternion().y(),
    //                                             margFilter_.getQuaternion().z())
    //                                             ;

    dm->data->com_vel_est = sp_->com_vel_est;
    dm->data->com_vel_raw = robot_->get_com_lin_vel();
    dm->data->imu_ang_vel_est = sp_->imu_ang_vel_est;
    dm->data->imu_ang_vel_raw = data->imu_frame_vel.head(3);
    dm->data->cam_est = sp_->cam_est;
    dm->data->cam_raw = robot_->hg.head(3);
    dm->data->icp = sp_->dcm.head(2);
    dm->data->icp_dot = sp_->dcm_vel.head(2);

    // save feet contact information
    dm->data->lfoot_contact = sp_->stance_foot == "l_foot_contact";
    dm->data->rfoot_contact = sp_->stance_foot == "r_foot_contact";
    dm->data->lf_contact = sp_->b_lf_contact;
    dm->data->rf_contact = sp_->b_rf_contact;

    dm->data->imu_accel_raw = data->imu_dvel / sp_->servo_dt;
    dm->data->imu_accel = base_acceleration_;

    dm->data->base_com_pos =
        data->base_com_pos; // TODO remove and pass directly from python
    dm->data->base_com_quat =
        data->base_com_quat; // TODO remove and pass directly from python
  }
}

void DracoKFStateEstimator::updateSupportState(DracoStateProvider *sp,
                                               SupportState &support_state) {
  if (sp->b_rf_contact && sp->b_lf_contact) {
    support_state = DOUBLE;
    return;
  }

  if (sp->b_lf_contact) {
    support_state = LEFT;
    return;
  }

  support_state = RIGHT;
}

Eigen::Matrix3d DracoKFStateEstimator::compute_world_to_base_rot(
    DracoSensorData *data, Eigen::Matrix3d rot_world_to_imu,
    bool use_marg_filter) {
  if (use_marg_filter) {
    //    margFilter_.filterUpdate(data->imu_frame_vel[0],
    //    data->imu_frame_vel[1], data->imu_frame_vel[2],
    //                              data->imu_accel[0], data->imu_accel[1],
    //                              data->imu_accel[2]);
    //    return margFilter_.getBaseRotation();
    return rot_world_to_imu * iso_imu_to_base_com_.linear();
  } else {
    return rot_world_to_imu * iso_imu_to_base_com_.linear();
  }
}

void DracoKFStateEstimator::ComputeDCM() {
  Eigen::Vector3d com_pos = robot_->get_com_pos();
  Eigen::Vector3d com_vel = sp_->com_vel_est;
  double dcm_omega = sqrt(9.81 / com_pos[2]);

  sp_->prev_dcm = sp_->dcm;
  sp_->dcm = com_pos + com_vel / dcm_omega;

  double alpha_vel = 0.1; // TODO Study this alpha value
  sp_->dcm_vel = alpha_vel * ((sp_->dcm - sp_->prev_dcm) / sp_->servo_dt) +
                 (1.0 - alpha_vel) * sp_->dcm_vel;
}
