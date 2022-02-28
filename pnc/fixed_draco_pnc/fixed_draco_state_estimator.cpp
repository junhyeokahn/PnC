#include "pnc/fixed_draco_pnc/fixed_draco_state_estimator.hpp"

#include "pnc/fixed_draco_pnc/fixed_draco_interface.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_provider.hpp"
#include "pnc/robot_system/robot_system.hpp"

FixedDracoStateEstimator::FixedDracoStateEstimator(RobotSystem *_robot) {
  util::PrettyConstructor(1, "FixedDracoStateEstimator");
  robot_ = _robot;
  sp_ = FixedDracoStateProvider::getStateProvider();

  iso_base_com_to_imu_ = robot_->get_link_iso("torso_link").inverse() *
                         robot_->get_link_iso("torso_imu");
  iso_base_joint_to_imu_.linear() = iso_base_com_to_imu_.linear();
  iso_base_joint_to_imu_.translation() =
      iso_base_com_to_imu_.translation() +
      robot_->get_link_iso("torso_link").linear() *
          robot_->get_base_local_com_pos();
}

FixedDracoStateEstimator::~FixedDracoStateEstimator() {}

void FixedDracoStateEstimator::initialize(FixedDracoSensorData *data) {
  this->update(data);
}

void FixedDracoStateEstimator::update(FixedDracoSensorData *_data) {

  // estimate base angular state from imu data
  Eigen::Matrix<double, 3, 3> rot_world_to_base =
      _data->imu_frame_iso.block(0, 0, 3, 3) *
      iso_base_joint_to_imu_.inverse().linear();
  Eigen::Quaternion<double> quat_world_to_base =
      Eigen::Quaternion<double>(rot_world_to_base);

  // update system
  robot_->update_system(Eigen::Vector3d::Zero(), quat_world_to_base,
                        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero(), quat_world_to_base,
                        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                        _data->joint_positions, _data->joint_velocities, true);

  // save current time step data
  if (sp_->count % sp_->save_freq == 0) {
    FixedDracoDataManager *dm =
        FixedDracoDataManager::GetFixedDracoDataManager();
    dm->data->joint_positions = robot_->get_q().tail(robot_->n_a);
    dm->data->joint_velocities = robot_->get_q_dot().tail(robot_->n_a);
    dm->data->base_joint_pos_est = Eigen::VectorXd::Zero(3);
    dm->data->base_joint_quat_est = Eigen::Matrix<double, 4, 1>(
        quat_world_to_base.w(), quat_world_to_base.x(), quat_world_to_base.y(),
        quat_world_to_base.z());
  }
}
