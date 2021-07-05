#pragma once

#include <map>
#include <string>

#include "configuration.hpp"
#include "utils/util.hpp"

class DracoStateProvider {
public:
  static DracoStateProvider *getStateProvider();
  ~DracoStateProvider() {}

  std::map<std::string, double> nominal_joint_pos;

  int state;
  int prev_state;

  std::string stance_foot;
  std::string prev_stance_foot;

  double curr_time;
  double servo_rate = 0.001;

  Eigen::Vector3d dcm;
  Eigen::Vector3d prev_dcm;
  Eigen::Vector3d dcm_vel;

  int b_rf_contact;
  int b_lf_contact;

  int planning_id = 0;

private:
  DracoStateProvider();
};
