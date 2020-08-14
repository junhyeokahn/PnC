#include <PnC/TrajectoryManager/DCMTrajectoryManager.hpp>

DCMTrajectoryManager::DCMTrajectoryManager(DCMPlanner* _dcm_planner,
                                           Task* _com_task,
                                           Task* _base_ori_task,
                                           RobotSystem* _robot, int _lfoot_idx,
                                           int _rfoot_idx)
    : TrajectoryManagerBase(_robot) {
  myUtils::pretty_constructor(2, "TrajectoryManager: DCM Planner");
  dcm_planner_ = _dcm_planner;
  com_task_ = _com_task;
  base_ori_task_ = _base_ori_task;
  lfoot_id_ = _lfoot_idx;
  rfoot_id_ = _rfoot_idx;

  des_dcm.setZero();
  des_dcm_vel.setZero();
  des_com_pos.setZero();
  des_com_vel.setZero();
  des_com_acc.setZero();
  des_quat.setIdentity();
  des_ang_vel.setZero();
  des_ang_acc.setZero();

  // Initialize step index
  resetStepIndex();

  // Initialize default parameters
  t_additional_init_transfer_ = 0.0;
  t_contact_transition_ = 0.45;
  t_swing_ = 1.0;

  // DCM walking parameters
  percentage_settle_ =
      0.99;  // percent to converge at the end of the trajectory
  alpha_ds_ =
      0.5;  // value between 0.0 and 1.0 for double support DCM interpolation
  nominal_com_height_ = 1.015;  // vertical m from stance foot

  // Nominal walking parameters
  nominal_footwidth_ = 0.27;
  nominal_forward_step_ = 0.25;
  nominal_backward_step_ = -0.25;
  nominal_turn_radians_ = M_PI / 4.0;
  nominal_strafe_distance_ = 0.125;

  // First step before alternating
  robot_side_first_ = RIGHT_ROBOT_SIDE;

  convertTemporalParamsToDCMParams();
}

DCMTrajectoryManager::~DCMTrajectoryManager() {}

void DCMTrajectoryManager::convertTemporalParamsToDCMParams() {
  // Fixed transforms
  t_ds_ = t_contact_transition_;  // double support polynomial transfer time
  t_ss_ = t_swing_;  // single support exponential interpolation  time
  // polynomial interpolation time during contact transition: t_transfer + t_ds
  // + (1-alpha*t_ds).
  t_transfer_init_ =
      t_additional_init_transfer_;  // additional transfer time offset
  t_transfer_mid_ =
      (alpha_ds_ - 1.0) * t_ds_;  // transfer time offset for midstep transfers
}

double DCMTrajectoryManager::getInitialContactTransferTime() {
  double t_initial_transfer_time =
      t_additional_init_transfer_ + t_ds_ +
      (1 - alpha_ds_) *
          t_ds_;  // the total initial transfer time before the foot swinng
  return t_initial_transfer_time;
}

// midstep transfer time before contact transition
double DCMTrajectoryManager::getMidStepContactTransferTime() {
  double t_midstep_transfer = t_ds_;
  return t_midstep_transfer;
}

// total time after landing the last step.
double DCMTrajectoryManager::getFinalContactTransferTime() {
  double t_final_transfer = t_ds_ + dcm_planner_->get_settle_time();
  return t_final_transfer;
}

double DCMTrajectoryManager::getSwingTime() { return t_ss_; }

double DCMTrajectoryManager::getNormalForceRampUpTime() {
  return alpha_ds_ * t_ds_;
}
double DCMTrajectoryManager::getNormalForceRampDownTime() {
  return (1.0 - alpha_ds_) * t_ds_;
}

void DCMTrajectoryManager::incrementStepIndex() { current_footstep_index_++; }
void DCMTrajectoryManager::resetStepIndex() { current_footstep_index_ = 0; }

// Updates the feet pose of the starting stance
void DCMTrajectoryManager::updateStartingStance() {
  Eigen::Vector3d lfoot_pos =
      robot_->getBodyNodeCoMIsometry(lfoot_id_).translation();
  Eigen::Quaterniond lfoot_ori(
      robot_->getBodyNodeCoMIsometry(lfoot_id_).linear());
  left_foot_stance_.setPosOriSide(lfoot_pos, lfoot_ori, LEFT_ROBOT_SIDE);

  Eigen::Vector3d rfoot_pos =
      robot_->getBodyNodeCoMIsometry(rfoot_id_).translation();
  Eigen::Quaterniond rfoot_ori(
      robot_->getBodyNodeCoMIsometry(rfoot_id_).linear());
  right_foot_stance_.setPosOriSide(rfoot_pos, rfoot_ori, RIGHT_ROBOT_SIDE);

  mid_foot_stance_.computeMidfeet(left_foot_stance_, right_foot_stance_,
                                  mid_foot_stance_);
}

// Updates the local footstep list (ie: footstep preview) for trajectory
// generation:
void DCMTrajectoryManager::updatePreview(const int max_footsteps_to_preview) {
  footstep_preview_list_.clear();
  for (int i = 0; i < max_footsteps_to_preview; i++) {
    if ((i + current_footstep_index_) < footstep_list_.size()) {
      footstep_preview_list_.push_back(
          footstep_list_[i + current_footstep_index_]);
    } else {
      break;
    }
  }
}

bool DCMTrajectoryManager::initialize(const double t_walk_start_in,
                                      const int transfer_type_in,
                                      const Eigen::Quaterniond& ori_start_in,
                                      const Eigen::Vector3d& dcm_pos_start_in,
                                      const Eigen::Vector3d& dcm_vel_start_in) {
  if (footstep_list_.size() == 0) {
    return false;
  }

  t_walk_start_ = t_walk_start_in;

  updateStartingStance();
  left_foot_start_ = left_foot_stance_;
  right_foot_start_ = right_foot_stance_;
  updatePreview(40);

  // If preview list is empty, don't update.
  if (footstep_preview_list_.size() == 0) {
    std::cout << "[DCMTrajectoryManager] ERROR. Footstep preview list "
                 "is empty."
              << std::endl;
    return false;
  }

  // std::cout << "[[DCM INITIALIZATION]]" << std::endl;
  // std::cout << "dcm" << std::endl;
  // std::cout << dcm_pos_start_in << std::endl;
  // std::cout << "dcm vel" << std::endl;
  // std::cout << dcm_vel_start_in << std::endl;

  // Set DCM reference
  dcm_planner_->setRobotMass(robot_->getRobotMass());
  dcm_planner_->setCoMHeight(nominal_com_height_);
  dcm_planner_->setInitialTime(t_walk_start_);
  dcm_planner_->setInitialOri(ori_start_in);

  // Set transfer time
  if (transfer_type_in == DCM_TRANSFER_TYPES::INITIAL) {
    dcm_planner_->t_transfer = t_transfer_init_;
  } else if (transfer_type_in == DCM_TRANSFER_TYPES::MIDSTEP) {
    dcm_planner_->t_transfer = t_transfer_mid_;
  }

  dcm_planner_->initialize_footsteps_rvrp(footstep_preview_list_,
                                          left_foot_start_, right_foot_start_,
                                          dcm_pos_start_in, dcm_vel_start_in);

  // Initialization successful
  return true;
}

void DCMTrajectoryManager::updateDCMTasksDesired(double current_time) {
  dcm_planner_->get_ref_dcm(current_time, des_dcm);
  dcm_planner_->get_ref_dcm_vel(current_time, des_dcm_vel);
  dcm_planner_->get_ref_com(current_time, des_com_pos);
  dcm_planner_->get_ref_com_vel(current_time, des_com_vel);
  dcm_planner_->get_ref_ori_ang_vel_acc(current_time, des_quat, des_ang_vel,
                                        des_ang_acc);

  Eigen::VectorXd des_quat_vec = Eigen::VectorXd::Zero(4);
  des_quat_vec << des_quat.w(), des_quat.x(), des_quat.y(), des_quat.z();

  com_task_->updateDesired(des_com_pos, des_com_vel, des_com_acc);
  base_ori_task_->updateDesired(des_quat_vec, des_ang_vel, des_ang_acc);
}

// TEST
// void DCMTrajectoryManager::updateDCMTasksDesired(double current_time) {
// dcm_planner_->get_ref_dcm(current_time, des_dcm);
// dcm_planner_->get_ref_dcm_vel(current_time, des_dcm_vel);
// dcm_planner_->get_ref_com(current_time, des_com_pos);
// dcm_planner_->get_ref_com_vel(current_time, des_com_vel);
// dcm_planner_->get_ref_ori_ang_vel_acc(current_time, des_quat, des_ang_vel,
// des_ang_acc);

// Eigen::VectorXd des_quat_vec = Eigen::VectorXd::Zero(4);
// des_quat_vec << des_quat.w(), des_quat.x(), des_quat.y(), des_quat.z();

// des_dcm[2] = des_com_pos[2];
// des_dcm_vel[2] = des_com_vel[2];
// com_task_->updateDesired(des_dcm, des_dcm_vel, Eigen::VectorXd::Zero(3));
// base_ori_task_->updateDesired(des_quat_vec, des_ang_vel, des_ang_acc);
//}
// TEST

bool DCMTrajectoryManager::nextStepRobotSide(int& robot_side) {
  if ((footstep_list_.size() > 0) &&
      (current_footstep_index_ < footstep_list_.size())) {
    // std::cout << "hello:" << std::endl;
    // footstep_list_[current_footstep_index_].printInfo();
    robot_side = footstep_list_[current_footstep_index_].robot_side;
    return true;
  } else {
    return false;
  }
}

bool DCMTrajectoryManager::noRemainingSteps() {
  if (current_footstep_index_ >= footstep_list_.size()) {
    return true;
  } else {
    return false;
  }
}

void DCMTrajectoryManager::alternateLeg() {
  if (robot_side_first_ == LEFT_ROBOT_SIDE) {
    robot_side_first_ = RIGHT_ROBOT_SIDE;
  } else {
    robot_side_first_ = LEFT_ROBOT_SIDE;
  }
}

void DCMTrajectoryManager::resetIndexAndClearFootsteps() {
  // Reset index and footstep list
  resetStepIndex();
  footstep_list_.clear();
}

void DCMTrajectoryManager::walkInPlace() {
  resetIndexAndClearFootsteps();
  populateStepInPlace(1, robot_side_first_);
  alternateLeg();
}
void DCMTrajectoryManager::walkForward() {
  resetIndexAndClearFootsteps();
  populateWalkForward(5, nominal_forward_step_);
  alternateLeg();
}
void DCMTrajectoryManager::walkBackward() {
  resetIndexAndClearFootsteps();
  populateWalkForward(5, nominal_backward_step_);
  alternateLeg();
}
void DCMTrajectoryManager::strafeLeft() {
  resetIndexAndClearFootsteps();
  populateStrafe(nominal_strafe_distance_, 2);
}
void DCMTrajectoryManager::strafeRight() {
  resetIndexAndClearFootsteps();
  populateStrafe(-nominal_strafe_distance_, 2);
}
void DCMTrajectoryManager::turnLeft() {
  resetIndexAndClearFootsteps();
  populateRotateTurn(nominal_turn_radians_, 2);
}
void DCMTrajectoryManager::turnRight() {
  resetIndexAndClearFootsteps();
  populateRotateTurn(-nominal_turn_radians_, 2);
}

// Footstep sequence primitives
// -----------------------------------------------------------
// Creates footstep in place
void DCMTrajectoryManager::populateStepInPlace(const int num_steps,
                                               const int robot_side_first) {
  updateStartingStance();  // Update the starting foot locations of the robot

  Footstep left_footstep = left_foot_stance_;
  Footstep right_footstep = right_foot_stance_;
  Footstep mid_footstep = mid_foot_stance_;

  int robot_side = robot_side_first;
  for (int i = 0; i < num_steps; i++) {
    // Square feet and switch sides
    if (robot_side == LEFT_ROBOT_SIDE) {
      left_footstep.setPosOri(
          mid_footstep.position +
              mid_footstep.R_ori *
                  Eigen::Vector3d(0, nominal_footwidth_ / 2.0, 0.0),
          mid_footstep.orientation);
      footstep_list_.push_back(left_footstep);
      robot_side = RIGHT_ROBOT_SIDE;
    } else {
      right_footstep.setPosOri(
          mid_footstep.position +
              mid_footstep.R_ori *
                  Eigen::Vector3d(0, -nominal_footwidth_ / 2.0, 0.0),
          mid_footstep.orientation);
      footstep_list_.push_back(right_footstep);
      robot_side = LEFT_ROBOT_SIDE;
    }
  }
}

// Populates the input footstep list with a predefined walking forward behavior
void DCMTrajectoryManager::populateWalkForward(const int num_steps,
                                               const double forward_distance) {
  updateStartingStance();  // Update the starting foot locations of the robot

  Footstep new_footstep;
  Footstep mid_footstep = mid_foot_stance_;

  int robot_side = LEFT_ROBOT_SIDE;
  for (int i = 0; i < num_steps; i++) {
    if (robot_side == LEFT_ROBOT_SIDE) {
      Eigen::Vector3d translate((i + 1) * forward_distance,
                                nominal_footwidth_ / 2.0, 0);
      new_footstep.setPosOriSide(
          mid_footstep.position + mid_footstep.R_ori * translate,
          mid_footstep.orientation, LEFT_ROBOT_SIDE);
      robot_side = RIGHT_ROBOT_SIDE;
    } else {
      Eigen::Vector3d translate((i + 1) * forward_distance,
                                -nominal_footwidth_ / 2.0, 0);
      new_footstep.setPosOriSide(
          mid_footstep.position + mid_footstep.R_ori * translate,
          mid_footstep.orientation, RIGHT_ROBOT_SIDE);
      robot_side = LEFT_ROBOT_SIDE;
    }
    footstep_list_.push_back(new_footstep);
  }

  // Add additional step forward to square the feet.
  if (robot_side == LEFT_ROBOT_SIDE) {
    Eigen::Vector3d translate(num_steps * forward_distance,
                              nominal_footwidth_ / 2.0, 0);
    new_footstep.setPosOriSide(
        mid_footstep.position + mid_footstep.R_ori * translate,
        mid_footstep.orientation, LEFT_ROBOT_SIDE);
  } else {
    Eigen::Vector3d translate(num_steps * forward_distance,
                              -nominal_footwidth_ / 2.0, 0);
    new_footstep.setPosOriSide(
        mid_footstep.position + mid_footstep.R_ori * translate,
        mid_footstep.orientation, RIGHT_ROBOT_SIDE);
  }
  footstep_list_.push_back(new_footstep);
}

// Take two steps to rotate at the specified radians. Repeat num_times
void DCMTrajectoryManager::populateRotateTurn(
    const double turn_radians_per_step, const int num_times) {
  updateStartingStance();  // Update the starting foot locations of the robot

  Eigen::Quaterniond foot_rotate(
      Eigen::AngleAxisd(turn_radians_per_step, Eigen::Vector3d::UnitZ()));

  Footstep left_footstep, right_footstep;
  Footstep mid_footstep = mid_foot_stance_;

  Footstep mid_footstep_rotated = mid_footstep;
  for (int i = 0; i < num_times; i++) {
    mid_footstep_rotated.setPosOri(mid_footstep.position,
                                   foot_rotate * mid_footstep.orientation);

    left_footstep.setPosOriSide(
        mid_footstep_rotated.position +
            mid_footstep_rotated.R_ori *
                Eigen::Vector3d(0, nominal_footwidth_ / 2.0, 0),
        mid_footstep_rotated.orientation, LEFT_ROBOT_SIDE);
    right_footstep.setPosOriSide(
        mid_footstep_rotated.position +
            mid_footstep_rotated.R_ori *
                Eigen::Vector3d(0, -nominal_footwidth_ / 2.0, 0),
        mid_footstep_rotated.orientation, RIGHT_ROBOT_SIDE);

    if (turn_radians_per_step > 0) {
      footstep_list_.push_back(left_footstep);
      footstep_list_.push_back(right_footstep);
    } else {
      footstep_list_.push_back(right_footstep);
      footstep_list_.push_back(left_footstep);
    }
    mid_footstep = mid_footstep_rotated;
  }
}

// Take two steps to strafe at the specified distance. Repeat num_times.
void DCMTrajectoryManager::populateStrafe(const double strafe_distance,
                                          const int num_times) {
  updateStartingStance();  // Update the starting foot locations of the robot

  // Strafe
  Footstep left_footstep, right_footstep;
  Footstep mid_footstep = mid_foot_stance_;
  Footstep mid_footstep_translated = mid_footstep;

  for (int i = 0; i < num_times; i++) {
    mid_footstep_translated.setPosOri(
        mid_footstep.position +
            mid_footstep.R_ori * Eigen::Vector3d(0.0, strafe_distance, 0.0),
        mid_footstep.orientation);

    left_footstep.setPosOriSide(
        mid_footstep_translated.position +
            mid_footstep_translated.R_ori *
                Eigen::Vector3d(0, nominal_footwidth_ / 2.0, 0),
        mid_footstep_translated.orientation, LEFT_ROBOT_SIDE);
    right_footstep.setPosOriSide(
        mid_footstep_translated.position +
            mid_footstep_translated.R_ori *
                Eigen::Vector3d(0, -nominal_footwidth_ / 2.0, 0),
        mid_footstep_translated.orientation, RIGHT_ROBOT_SIDE);

    if (strafe_distance > 0) {
      // Left strafe
      footstep_list_.push_back(left_footstep);
      footstep_list_.push_back(right_footstep);
    } else {
      // Right strafe
      footstep_list_.push_back(right_footstep);
      footstep_list_.push_back(left_footstep);
    }
    mid_footstep = mid_footstep_translated;
  }
}

void DCMTrajectoryManager::paramInitialization(const YAML::Node& node) {
  // void setCoMHeight(double z_vrp_in); // Sets the desired CoM Height
  // Load Custom Params ----------------------------------
  try {
    // Load DCM Parameters
    myUtils::readParameter(node, "nominal_com_height", nominal_com_height_);
    myUtils::readParameter(node, "t_additional_init_transfer",
                           t_additional_init_transfer_);
    myUtils::readParameter(node, "t_contact_transition", t_contact_transition_);
    myUtils::readParameter(node, "t_swing", t_swing_);
    myUtils::readParameter(node, "percentage_settle", percentage_settle_);
    myUtils::readParameter(node, "alpha_ds", alpha_ds_);

    // Load Walking Primitives Parameters
    myUtils::readParameter(node, "nominal_footwidth", nominal_footwidth_);
    myUtils::readParameter(node, "nominal_forward_step", nominal_forward_step_);
    myUtils::readParameter(node, "nominal_backward_step",
                           nominal_backward_step_);
    myUtils::readParameter(node, "nominal_turn_radians", nominal_turn_radians_);
    myUtils::readParameter(node, "nominal_strafe_distance",
                           nominal_strafe_distance_);

  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }
  // Convert temporal parameters to DCM parameters
  convertTemporalParamsToDCMParams();

  // Set DCM parameters
  dcm_planner_->t_transfer = t_transfer_init_;  // Time varying after every step
  dcm_planner_->t_ds = t_ds_;
  dcm_planner_->t_ss = t_ss_;
  dcm_planner_->percentage_settle = percentage_settle_;
  dcm_planner_->alpha_ds = alpha_ds_;
}

void DCMTrajectoryManager::saveSolution(const std::string& file_name) {
  try {
    double t_start = dcm_planner_->getInitialTime();
    double t_end = t_start + dcm_planner_->get_total_trajectory_time();
    double t_step(0.01);
    int n_eval = std::floor((t_end - t_start) / t_step);

    YAML::Node cfg;

    // =====================================================================
    // Temporal Parameters
    // =====================================================================

    cfg["temporal_parameters"]["initial_time"] = t_start;
    cfg["temporal_parameters"]["final_time"] = t_end;
    cfg["temporal_parameters"]["time_step"] = t_step;
    cfg["temporal_parameters"]["t_ds"] = dcm_planner_->t_ds;
    cfg["temporal_parameters"]["t_ss"] = dcm_planner_->t_ss;
    cfg["temporal_parameters"]["t_transfer"] = dcm_planner_->t_transfer;

    // =====================================================================
    // Contact Information
    // =====================================================================
    Eigen::MatrixXd curr_rfoot_pos = Eigen::MatrixXd::Zero(1, 3);
    Eigen::MatrixXd curr_rfoot_quat = Eigen::MatrixXd::Zero(1, 4);
    Eigen::MatrixXd curr_lfoot_pos = Eigen::MatrixXd::Zero(1, 3);
    Eigen::MatrixXd curr_lfoot_quat = Eigen::MatrixXd::Zero(1, 4);
    for (int i = 0; i < 3; ++i) {
      curr_rfoot_pos(0, i) = right_foot_start_.position(i);
      curr_lfoot_pos(0, i) = left_foot_start_.position(i);
    }
    curr_rfoot_quat(0, 0) = right_foot_start_.orientation.w();
    curr_rfoot_quat(0, 1) = right_foot_start_.orientation.x();
    curr_rfoot_quat(0, 2) = right_foot_start_.orientation.y();
    curr_rfoot_quat(0, 3) = right_foot_start_.orientation.z();

    curr_lfoot_quat(0, 0) = left_foot_start_.orientation.w();
    curr_lfoot_quat(0, 1) = left_foot_start_.orientation.x();
    curr_lfoot_quat(0, 2) = left_foot_start_.orientation.y();
    curr_lfoot_quat(0, 3) = left_foot_start_.orientation.z();

    int n_rf(0);
    int n_lf(0);
    for (int i = 0; i < footstep_list_.size(); ++i) {
      if (footstep_list_[i].robot_side == LEFT_ROBOT_SIDE) {
        n_lf += 1;
      } else {
        n_rf += 1;
      }
    }
    Eigen::MatrixXd rfoot_pos = Eigen::MatrixXd::Zero(n_rf, 3);
    Eigen::MatrixXd rfoot_quat = Eigen::MatrixXd::Zero(n_rf, 4);
    Eigen::MatrixXd lfoot_pos = Eigen::MatrixXd::Zero(n_lf, 3);
    Eigen::MatrixXd lfoot_quat = Eigen::MatrixXd::Zero(n_lf, 4);
    int rf_id(0);
    int lf_id(0);
    for (int i = 0; i < footstep_list_.size(); ++i) {
      if (footstep_list_[i].robot_side == RIGHT_ROBOT_SIDE) {
        for (int j = 0; j < 3; ++j) {
          rfoot_pos(rf_id, j) = footstep_list_[i].position(j);
        }
        rfoot_quat(rf_id, 0) = footstep_list_[i].orientation.w();
        rfoot_quat(rf_id, 1) = footstep_list_[i].orientation.x();
        rfoot_quat(rf_id, 2) = footstep_list_[i].orientation.y();
        rfoot_quat(rf_id, 3) = footstep_list_[i].orientation.z();
        rf_id += 1;
      } else {
        for (int j = 0; j < 3; ++j) {
          lfoot_pos(lf_id, j) = footstep_list_[i].position(j);
        }
        lfoot_quat(lf_id, 0) = footstep_list_[i].orientation.w();
        lfoot_quat(lf_id, 1) = footstep_list_[i].orientation.x();
        lfoot_quat(lf_id, 2) = footstep_list_[i].orientation.y();
        lfoot_quat(lf_id, 3) = footstep_list_[i].orientation.z();
        lf_id += 1;
      }
    }

    cfg["contact"]["curr_right_foot"]["pos"] = curr_rfoot_pos;
    cfg["contact"]["curr_right_foot"]["ori"] = curr_rfoot_quat;
    cfg["contact"]["curr_left_foot"]["pos"] = curr_lfoot_pos;
    cfg["contact"]["curr_left_foot"]["ori"] = curr_lfoot_quat;
    cfg["contact"]["right_foot"]["pos"] = rfoot_pos;
    cfg["contact"]["right_foot"]["ori"] = rfoot_quat;
    cfg["contact"]["left_foot"]["pos"] = lfoot_pos;
    cfg["contact"]["left_foot"]["ori"] = lfoot_quat;

    // =====================================================================
    // Reference Trajectory
    // =====================================================================
    Eigen::MatrixXd dcm_pos_ref = Eigen::MatrixXd::Zero(n_eval, 3);
    Eigen::MatrixXd dcm_vel_ref = Eigen::MatrixXd::Zero(n_eval, 3);
    Eigen::MatrixXd com_pos_ref = Eigen::MatrixXd::Zero(n_eval, 3);
    Eigen::MatrixXd com_vel_ref = Eigen::MatrixXd::Zero(n_eval, 3);
    Eigen::MatrixXd vrp_ref = Eigen::MatrixXd::Zero(n_eval, 3);
    Eigen::MatrixXd t_traj = Eigen::MatrixXd::Zero(n_eval, 1);

    double t(t_start);
    Eigen::Vector3d v3;
    for (int i = 0; i < n_eval; ++i) {
      t_traj(i, 0) = t;
      dcm_planner_->get_ref_dcm(t, v3);
      for (int j = 0; j < 3; ++j) {
        dcm_pos_ref(i, j) = v3(j);
      }
      dcm_planner_->get_ref_dcm_vel(t, v3);
      for (int j = 0; j < 3; ++j) {
        dcm_vel_ref(i, j) = v3(j);
      }
      dcm_planner_->get_ref_dcm_vel(t, v3);
      for (int j = 0; j < 3; ++j) {
        dcm_vel_ref(i, j) = v3(j);
      }
      dcm_planner_->get_ref_com(t, v3);
      for (int j = 0; j < 3; ++j) {
        com_pos_ref(i, j) = v3(j);
      }
      dcm_planner_->get_ref_com_vel(t, v3);
      for (int j = 0; j < 3; ++j) {
        com_vel_ref(i, j) = v3(j);
      }
      dcm_planner_->get_ref_r_vrp(t, v3);
      for (int j = 0; j < 3; ++j) {
        vrp_ref(i, j) = v3(j);
      }
      t += t_step;
    }

    cfg["reference"]["dcm_pos"] = dcm_pos_ref;
    cfg["reference"]["dcm_vel"] = dcm_vel_ref;
    cfg["reference"]["com_pos"] = com_pos_ref;
    cfg["reference"]["com_vel"] = com_vel_ref;
    cfg["reference"]["vrp"] = vrp_ref;
    cfg["reference"]["time"] = t_traj;

    std::string full_path = THIS_COM + std::string("ExperimentData/") +
                            file_name + std::string(".yaml");
    std::ofstream file_out(full_path);
    file_out << cfg;

  } catch (YAML::ParserException& e) {
    std::cout << e.what() << std::endl;
  }
}
