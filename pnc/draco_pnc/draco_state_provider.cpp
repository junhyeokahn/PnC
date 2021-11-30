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
}
