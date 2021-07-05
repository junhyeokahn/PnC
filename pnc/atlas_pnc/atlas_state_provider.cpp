#include <pnc/atlas_pnc/atlas_state_provider.hpp>
#include <pnc/robot_system/robot_system.hpp>

AtlasStateProvider *AtlasStateProvider::getStateProvider(RobotSystem *_robot) {
  static AtlasStateProvider state_provider_(_robot);
  return &state_provider_;
}

AtlasStateProvider::AtlasStateProvider(RobotSystem *_robot) {
  util::PrettyConstructor(1, "Atlas State Provider");

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
