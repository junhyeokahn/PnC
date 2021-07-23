#include "pnc/fixed_draco_pnc/fixed_draco_state_provider.hpp"

//#include "Utils/IO/DataManager.hpp"

FixedDracoStateProvider *FixedDracoStateProvider::getStateProvider() {
  static FixedDracoStateProvider state_provider_;
  return &state_provider_;
}

FixedDracoStateProvider::FixedDracoStateProvider() {
  util::PrettyConstructor(1, "FixedDracoStateProvider");

  state = 0;
  prev_state = 0;

  save_freq = 50;
  count = 0;
  curr_time = 0.;

  b_smoothing_cmd = false;
  smoothing_start_time = 0.;
  smoothing_duration = 3.;
}
