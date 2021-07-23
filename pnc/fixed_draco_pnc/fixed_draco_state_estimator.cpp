#include "pnc/fixed_draco_pnc/fixed_draco_state_estimator.hpp"

#include "pnc/fixed_draco_pnc/fixed_draco_interface.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_provider.hpp"
#include "pnc/robot_system/robot_system.hpp"

FixedDracoStateEstimator::FixedDracoStateEstimator(RobotSystem *_robot) {
  util::PrettyConstructor(1, "FixedDracoStateEstimator");
  robot_ = _robot;
  sp_ = FixedDracoStateProvider::getStateProvider();

  slack_quat_.setIdentity();
}

FixedDracoStateEstimator::~FixedDracoStateEstimator() {}

void FixedDracoStateEstimator::initialize(FixedDracoSensorData *data) {
  this->update(data);
}

void FixedDracoStateEstimator::update(FixedDracoSensorData *data) {

  // update system
  robot_->update_system(Eigen::Vector3d::Zero(), slack_quat_,
                        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero(), slack_quat_,
                        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                        data->joint_positions, data->joint_velocities, true);

  // save current time step data

  if (sp_->count % sp_->save_freq == 0) {
    FixedDracoDataManager *dm =
        FixedDracoDataManager::GetFixedDracoDataManager();
    dm->data->joint_positions = robot_->get_q().tail(robot_->n_a);
    dm->data->joint_velocities = robot_->get_q_dot().tail(robot_->n_a);
  }
}
