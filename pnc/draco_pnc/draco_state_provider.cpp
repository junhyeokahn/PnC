#include "pnc/draco_pnc/draco_state_provider.hpp"

//#include "Utils/IO/DataManager.hpp"

DracoStateProvider *DracoStateProvider::getStateProvider() {
  static DracoStateProvider state_provider_;
  return &state_provider_;
}

DracoStateProvider::DracoStateProvider() {
  util::PrettyConstructor(1, "DracoStateProvider");

  state = 0;
  prev_state = 0;

  stance_foot = "l_foot_contact";
  prev_stance_foot = "l_foot_contact";

  nominal_base_quat = Eigen::Quaternion<double>::Identity();

  save_freq = 50;
  count = 0;
  curr_time = 0.;

  dcm.setZero();
  prev_dcm.setZero();
  dcm_vel.setZero();

  com_vel_est.setZero();
  cam_est.setZero();
  imu_ang_vel_est.setZero();

  // 0: no contact, 1: contact
  b_rf_contact = true;
  b_lf_contact = true;

  floating_jidx = {0, 1, 2, 3, 4, 5};

  nominal_stance_foot_iso.setIdentity();

  b_state_estimator_on = false;
  des_com_pos_in_standup.setZero();

  nominal_lfoot_iso.setIdentity();
  nominal_rfoot_iso.setIdentity();

  des_com_pos_in_ds_move.setZero();

  des_rfoot_in_foot_lifting.setZero();
  des_lfoot_in_foot_lifting.setZero();

  des_rfoot_ori_foot_lifting.setIdentity();
  des_lfoot_ori_foot_lifting.setIdentity();
}

void DracoStateProvider::setStateEstimator(bool flag) { this->b_state_estimator_on = flag; }

bool DracoStateProvider::isStateEstimatorOn() { return b_state_estimator_on; }