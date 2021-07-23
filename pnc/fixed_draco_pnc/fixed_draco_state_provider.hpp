#pragma once

#include <map>
#include <string>

#include "configuration.hpp"
#include "utils/util.hpp"

class FixedDracoStateProvider {
public:
  static FixedDracoStateProvider *getStateProvider();
  ~FixedDracoStateProvider() {}

  // should be set outside
  std::map<std::string, double> nominal_joint_pos;

  // finite state machine
  int state;
  int prev_state;

  // timing
  int count;
  double curr_time;

  // should be set outside
  int save_freq;
  double servo_dt = 0.001;

  // command smoothing for hardware experiment
  bool b_smoothing_cmd;
  double smoothing_start_time;
  double smoothing_duration;

private:
  FixedDracoStateProvider();
};
