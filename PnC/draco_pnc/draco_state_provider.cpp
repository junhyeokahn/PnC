#include "PnC/draco_pnc/draco_state_provider.hpp"

#include "Utils/IO/DataManager.hpp"

DracoStateProvider *DracoStateProvider::getStateProvider() {
  static DracoStateProvider state_provider_;
  return &state_provider_;
}

DracoStateProvider::DracoStateProvider() {
  myUtils::pretty_constructor(1, "Draco State Provider");

  state = 0;
  prev_state = 0;

  curr_time = 0.;

  dcm.setZero();
  prev_dcm.setZero();
  dcm_vel.setZero();

  // 0: no contact, 1: contact
  b_rf_contact = 0;
  b_lf_contact = 0;
}
