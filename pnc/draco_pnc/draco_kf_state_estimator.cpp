#include "draco_kf_state_estimator.hpp"

DracoKFStateEstimator::DracoKFStateEstimator(RobotSystem *_robot) {
  util::PrettyConstructor(1, "DracoKFStateEstimator");
  robot_ = _robot;
  sp_ = DracoStateProvider::getStateProvider();

  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");

  iso_imu_to_base_com_ = robot_->get_link_iso("torso_imu").inverse() *
                        robot_->get_link_iso("torso_link");

  x_hat_.setZero();
  system_model_.initialize(deltat);
  double gravity = -9.81; //TODO get from somewhere else
  base_pose_model_.initialize(gravity);
  rot_world_to_base.setZero();

  b_first_visit_ = true;
}

DracoKFStateEstimator::~DracoKFStateEstimator() {
}

void DracoKFStateEstimator::initialize(DracoSensorData *data) {
    this->update(data);
}

void DracoKFStateEstimator::update(DracoSensorData *data) {

  // continue only when state estimator is on
  if (!(sp_->isStateEstimatorOn()))
    return;

  // estimate 0_R_b
  margFilter_.filterUpdate(data->imu_frame_vel[0], data->imu_frame_vel[1], data->imu_frame_vel[2],
                            data->imu_accel[0], data->imu_accel[1], data->imu_accel[2]);
//  rot_world_to_base = margFilter_.getBaseRotation();
  rot_world_to_base = data->imu_frame_iso.block(0, 0, 3, 3) *
          iso_imu_to_base_com_.linear();

  // use Kalman filter to estimate
  // [0_pos_b, 0_vel_b, 0_pos_LF, 0_pos_RF]
  if (b_first_visit_) {
    Eigen::Vector3d torso_frame = Eigen::Vector3d::Zero();
    torso_frame = robot_->get_link_iso("torso_link").translation() - rot_world_to_base * robot_->get_base_local_com_pos();
    x_hat_.initialize(torso_frame,
                      robot_->get_link_iso("l_foot_contact"),
                      robot_->get_link_iso("r_foot_contact"));
    kalman_filter_.init(x_hat_);
    b_first_visit_ = false;
  }
  base_pose_model_.packAccelerationInput(rot_world_to_base, data->imu_accel, accelerometer_input_);
  x_hat_ = kalman_filter_.predict(system_model_, accelerometer_input_);

  // update contact
  if (data->b_rf_contact) {
      sp_->b_rf_contact = true;
  } else {
      sp_->b_rf_contact = false;
  }

  if (data->b_lf_contact) {
      sp_->b_lf_contact = true;
  } else {
      sp_->b_lf_contact = false;
  }

//    this->_update_dcm();
  // Note: update estimator assuming at least one foot is on the ground
  if (data->b_lf_contact) {
    base_pose_model_.update_position_from_lfoot(robot_,"l_foot_contact", "torso_link", base_estimate_);
  }
  if (data->b_rf_contact) {
    base_pose_model_.update_position_from_rfoot(robot_,"r_foot_contact", "torso_link", base_estimate_);
  }
  x_hat_ = kalman_filter_.update(base_pose_model_, base_estimate_);
  // values computed by linear KF estimator
  Eigen::Vector3d base_position_estimate( x_hat_.base_pos_x(),  x_hat_.base_pos_y(),  x_hat_.base_pos_z());
  Eigen::Vector3d base_velocity_estimate( x_hat_.base_vel_x(),  x_hat_.base_vel_y(),  x_hat_.base_vel_z());

//  data->base_com_quat = Eigen::Vector4d(margFilter_.getQuaternion().w(),
//                                        margFilter_.getQuaternion().x(),
//                                        margFilter_.getQuaternion().y(),
//                                        margFilter_.getQuaternion().z());

//  robot_->update_system(
//          base_com_pos, margFilter_.getQuaternion(),
//          base_velocity_estimate, data->imu_frame_vel.head(3), base_position_estimate,
//          margFilter_.getQuaternion(), base_velocity_estimate,
//          data->imu_frame_vel.head(3), data->joint_positions,
//          data->joint_velocities, true);


  // save current time step data
  if (sp_->count % sp_->save_freq == 0) {
    DracoDataManager *dm = DracoDataManager::GetDracoDataManager();
    dm->data->base_pos_kf = base_position_estimate;
    dm->data->base_vel_kf = base_velocity_estimate;
    dm->data->base_euler_kf = util::QuatToEulerZYX(Eigen::Quaterniond(rot_world_to_base));
    dm->data->base_quat_kf = Eigen::Vector4d(margFilter_.getQuaternion().w(),
                                             margFilter_.getQuaternion().x(),
                                             margFilter_.getQuaternion().y(),
                                             margFilter_.getQuaternion().z()) ;

    dm->data->base_com_pos = data->base_com_pos;
    dm->data->base_com_quat = data->base_com_quat;
  }
}
