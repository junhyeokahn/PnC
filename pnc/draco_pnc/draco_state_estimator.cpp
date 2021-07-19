#include "pnc/draco_pnc/draco_state_estimator.hpp"

#include "pnc/draco_pnc/draco_interface.hpp"
#include "pnc/draco_pnc/draco_state_provider.hpp"
#include "pnc/robot_system/robot_system.hpp"

DracoStateEstimator::DracoStateEstimator(RobotSystem *_robot) {
  util::PrettyConstructor(1, "DracoStateEstimator");
  robot_ = _robot;
  sp_ = DracoStateProvider::getStateProvider();

  iso_base_com_to_imu_ = robot_->get_link_iso("torso_link").inverse() *
                         robot_->get_link_iso("torso_imu");
  iso_base_joint_to_imu_.linear() = iso_base_com_to_imu_.linear();
  iso_base_joint_to_imu_.translation() =
      iso_base_com_to_imu_.translation() +
      robot_->get_link_iso("torso_link").linear() *
          robot_->get_base_local_com_pos();

  global_linear_offset_.setZero();
  prev_base_joint_pos_.setZero();
  prev_base_com_pos_.setZero();
}

DracoStateEstimator::~DracoStateEstimator() {}

void DracoStateEstimator::initialize(DracoSensorData *data) {
  sp_->nominal_joint_pos = data->joint_positions;
  // this->update_debug(data);
  this->update(data);
}

void DracoStateEstimator::update_debug(DracoSensorData *_data) {
  Eigen::Quaternion<double> base_com_quat(
      _data->base_com_quat[0], _data->base_com_quat[1], _data->base_com_quat[2],
      _data->base_com_quat[3]);

  robot_->update_system(_data->base_com_pos, base_com_quat,
                        _data->base_com_lin_vel, _data->base_com_ang_vel,
                        _data->base_joint_pos, base_com_quat,
                        _data->base_joint_lin_vel, _data->base_joint_ang_vel,
                        _data->joint_positions, _data->joint_velocities, true);

  if (sp_->count % sp_->save_freq == 0) {
    DracoDataManager *dm = DracoDataManager::GetDracoDataManager();
    dm->data->joint_positions = robot_->get_q().tail(robot_->n_a);
    dm->data->joint_velocities = robot_->get_q_dot().tail(robot_->n_a);
    dm->data->base_joint_pos = _data->base_joint_pos;
    dm->data->base_joint_quat = _data->base_com_quat;
  }
  this->ComputeDCM();
}

void DracoStateEstimator::update(DracoSensorData *data) {

  // estimate base angular state from imu data
  Eigen::Matrix<double, 3, 3> rot_world_to_base =
      data->imu_frame_iso.block(0, 0, 3, 3) *
      iso_base_joint_to_imu_.inverse().linear();

  // update system without base linear states
  robot_->update_system(
      Eigen::Vector3d::Zero(), Eigen::Quaternion<double>(rot_world_to_base),
      Eigen::Vector3d::Zero(), data->imu_frame_vel.head(3),
      Eigen::Vector3d::Zero(), Eigen::Quaternion<double>(rot_world_to_base),
      Eigen::Vector3d::Zero(), data->imu_frame_vel.head(3),
      data->joint_positions, data->joint_velocities, true);

  // estimate base linear states
  Eigen::Vector3d foot_pos, foot_vel;
  if (sp_->stance_foot == "l_foot_contact") {
    foot_pos = robot_->get_link_iso("l_foot_contact").translation();
    foot_vel = robot_->get_link_vel("l_foot_contact").tail(3);
  } else if (sp_->stance_foot == "r_foot_contact") {
    foot_pos = robot_->get_link_iso("r_foot_contact").translation();
    foot_vel = robot_->get_link_vel("r_foot_contact").tail(3);
  } else {
    assert(false);
  }

  if (sp_->stance_foot != sp_->prev_stance_foot) {
    Eigen::Vector3d new_stance_foot = foot_pos;
    Eigen::Vector3d old_stance_foot;
    if (sp_->prev_stance_foot == "r_foot_contact") {
      old_stance_foot = robot_->get_link_iso("r_foot_contact").translation();
    } else if (sp_->prev_stance_foot == "l_foot_contact") {
      old_stance_foot = robot_->get_link_iso("l_foot_contact").translation();
    } else {
      assert(false);
    }
    Eigen::Vector3d old_to_new = new_stance_foot - old_stance_foot;
    global_linear_offset_ += old_to_new;
  }

  Eigen::Vector3d base_joint_pos = global_linear_offset_ - foot_pos;
  Eigen::Vector3d base_com_pos =
      base_joint_pos + rot_world_to_base * robot_->get_base_local_com_pos();

  static bool b_first_visit(true);
  if (b_first_visit) {
    prev_base_joint_pos_ = base_joint_pos;
    prev_base_com_pos_ = base_com_pos;
    b_first_visit = false;
  }
  Eigen::Vector3d base_joint_lin_vel =
      (base_joint_pos - prev_base_joint_pos_) / sp_->servo_rate;
  Eigen::Vector3d base_com_lin_vel =
      (base_com_pos - prev_base_com_pos_) / sp_->servo_rate;
  // Eigen::Vector3d base_joint_vel = -foot_vel; // TODO: also use kinematics

  // update system with base linear states
  robot_->update_system(
      base_com_pos, Eigen::Quaternion<double>(rot_world_to_base),
      base_com_lin_vel, data->imu_frame_vel.head(3), base_joint_pos,
      Eigen::Quaternion<double>(rot_world_to_base), base_joint_lin_vel,
      data->imu_frame_vel.head(3), data->joint_positions,
      data->joint_velocities, true);

  // update dcm
  this->ComputeDCM();

  // update contact
  if (data->b_rf_contact) {
    sp_->b_rf_contact = 1;
  } else {
    sp_->b_rf_contact = 0;
  }

  if (data->b_lf_contact) {
    sp_->b_lf_contact = 1;
  } else {
    sp_->b_lf_contact = 0;
  }

  // save current time step data
  sp_->prev_stance_foot = sp_->stance_foot;
  prev_base_joint_pos_ = base_joint_pos;
  prev_base_com_pos_ = base_com_pos;

  if (sp_->count % sp_->save_freq == 0) {
    DracoDataManager *dm = DracoDataManager::GetDracoDataManager();
    dm->data->joint_positions = robot_->get_q().tail(robot_->n_a);
    dm->data->joint_velocities = robot_->get_q_dot().tail(robot_->n_a);
    dm->data->base_joint_pos = base_joint_pos;
    Eigen::Quaternion<double> quat =
        Eigen::Quaternion<double>(rot_world_to_base);
    dm->data->base_joint_quat =
        Eigen::Matrix<double, 4, 1>(quat.w(), quat.x(), quat.y(), quat.z());
  }
}

void DracoStateEstimator::ComputeDCM() {
  Eigen::Vector3d com_pos = robot_->get_com_pos();
  Eigen::Vector3d com_vel = robot_->get_com_lin_vel();
  double dcm_omega = sqrt(9.81 / com_pos[2]);

  sp_->prev_dcm = sp_->dcm;
  sp_->dcm = com_pos + com_vel / dcm_omega;

  double alpha_vel = 0.1; // TODO Study this alpha value
  sp_->dcm_vel = alpha_vel * ((sp_->dcm - sp_->prev_dcm) / sp_->servo_rate) +
                 (1.0 - alpha_vel) * sp_->dcm_vel;
}
