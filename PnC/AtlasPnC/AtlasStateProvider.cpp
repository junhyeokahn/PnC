#include <PnC/AtlasPnC/AtlasStateProvider.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <Utils/IO/DataManager.hpp>

AtlasStateProvider *AtlasStateProvider::getStateProvider(RobotSystem *_robot) {
  static AtlasStateProvider state_provider_(_robot);
  return &state_provider_;
}

AtlasStateProvider::AtlasStateProvider(RobotSystem *_robot) {
  myUtils::pretty_constructor(1, "Atlas State Provider");

  robot_ = _robot;

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
