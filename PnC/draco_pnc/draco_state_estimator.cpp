#include <PnC/draco_pnc/draco_state_estimator.hpp>

#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/draco_pnc/draco_interface.hpp>
#include <PnC/draco_pnc/draco_state_provider.hpp>
#include <Utils/Math/MathUtilities.hpp>

DracoStateEstimator::DracoStateEstimator(RobotSystem *_robot) {
  robot_ = _robot;
  sp_ = DracoStateProvider::getStateProvider();

  iso_base_com_to_imu_ = robot_->get_link_iso("torso_link").inverse() *
                         robot_->get_link_iso("torso_imu");
  iso_base_joint_to_imu_ = iso_base_com_to_imu_;
  iso_base_joint_to_imu_.translation() += robot_->get_base_local_com_pos();
}

DracoStateEstimator::~DracoStateEstimator() {}

void DracoStateEstimator::initialize(DracoSensorData *data) {
  sp_->nominal_joint_pos = data->joint_positions;
  this->update(data);
}

void DracoStateEstimator::update(DracoSensorData *data) {

  // estimate base states from imu data
  Eigen::Isometry3d iso_world_to_imu;
  iso_world_to_imu.linear() = data->imu_frame_iso.block(0, 0, 3, 3);
  iso_world_to_imu.translation() = data->imu_frame_iso.block(0, 3, 3, 1);

  Eigen::Isometry3d iso_world_to_base_joint =
      iso_world_to_imu * iso_base_joint_to_imu_.inverse();
  Eigen::Isometry3d iso_world_to_base_com =
      iso_world_to_imu * iso_base_com_to_imu_.inverse();

  Eigen::Matrix<double, 6, 1> imu_frame_vel_at_imu_frame =
      myUtils::block_diag(iso_world_to_imu.linear().transpose(),
                          iso_world_to_imu.linear().transpose()) *
      data->imu_frame_vel;

  Eigen::Matrix<double, 6, 1> base_joint_vel =
      myUtils::block_diag(iso_world_to_base_joint.linear(),
                          iso_world_to_base_joint.linear()) *
      (myUtils::Adjoint(iso_base_joint_to_imu_.linear(),
                        iso_base_joint_to_imu_.translation()) *
       imu_frame_vel_at_imu_frame);
  Eigen::Matrix<double, 6, 1> base_com_vel =
      myUtils::block_diag(iso_world_to_base_com.linear(),
                          iso_world_to_base_com.linear()) *
      (myUtils::Adjoint(iso_base_com_to_imu_.linear(),
                        iso_base_com_to_imu_.translation()) *
       imu_frame_vel_at_imu_frame);

  // update system
  robot_->update_system(
      iso_world_to_base_com.translation(),
      Eigen::Quaternion<double>(iso_world_to_base_com.linear()),
      base_com_vel.tail(3), base_com_vel.head(3),
      iso_world_to_base_joint.translation(),
      Eigen::Quaternion<double>(iso_world_to_base_joint.linear()),
      base_joint_vel.tail(3), base_joint_vel.head(3), data->joint_positions,
      data->joint_velocities, true);

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

  this->_update_dcm();
}

void DracoStateEstimator::_update_dcm() {
  Eigen::Vector3d com_pos = robot_->get_com_pos();
  Eigen::Vector3d com_vel = robot_->get_com_lin_vel();
  double dcm_omega = sqrt(9.81 / com_pos[2]);

  sp_->prev_dcm = sp_->dcm;
  sp_->dcm = com_pos + com_vel / dcm_omega;

  double alpha_vel = 0.1; // TODO Study this alpha value
  sp_->dcm_vel = alpha_vel * ((sp_->dcm - sp_->prev_dcm) / sp_->servo_rate) +
                 (1.0 - alpha_vel) * sp_->dcm_vel;
}
