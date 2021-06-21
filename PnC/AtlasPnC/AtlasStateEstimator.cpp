#include <PnC/AtlasPnC/AtlasStateEstimator.hAtlasStateEstimator.hpp>

AtlasStateEstimator::AtlasStateEstimator(RobotSystem *_robot) {
  robot_ = _robot;
  sp_ = AtlasStateProvider::getStateProvider(robot_);
}

AtlasStateEstimator::~AtlasStateEstimator() {}

void AtlasStateEstimator::initialization(AtlasSensorData *data) {
  sp_->nominal_joint_pos = data->joint_positions;
  this->update(data);
}

void AtlasStateEstimator::update(AtlasSensorData *data) {
  robot_->update_system(data->base_com_pos, data->base_com_quat,
                        data->base_com_lin_vel, data->base_com_ang_vel,
                        data->base_joint_pos, data->base_joint_quat,
                        data->base_joint_lin_vel, data->base_joint_ang_vel,
                        data->joint_pos, data->joint_vel);

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

void AtlasStateEstimator::_update_dcm() {
  Eigen::Vector3d com_pos = robot_->get_com_pos();
  Eigen::Vector3d com_vel = robot_->get_com_lin_vel();
  double dcm_omega = sqrt(9.81 / com_pos[2]);

  sp_->prev_dcm = sp_->dcm;
  sp_->dcm = com_pos + com_vel / dcm_omega;

  double alpha_vel = 0.1; // TODO Study this alpha value
  sp_->dcm_vel = alpha_vel * ((sp_->dcm - sp_->prev_dcm) / sp_->dt) +
                 (1.0 - alpha_vel) * sp_->dcm_vel;
}
