#pragma once

#include <map>
#include <string>

#include "configuration.hpp"
#include "utils/util.hpp"

class DracoStateProvider {
public:
  static DracoStateProvider *getStateProvider();
  ~DracoStateProvider() {}

  // should be set outside
  std::map<std::string, double> nominal_joint_pos;

  // finite state machine
  int state;
  int prev_state;

  // contact info << used in pos estimate
  std::string stance_foot;
  std::string prev_stance_foot;

  // timing
  int count;
  double curr_time;

  // should be set outside
  int save_freq;
  double servo_dt = 0.001;

  Eigen::Vector3d dcm;
  Eigen::Vector3d prev_dcm;
  Eigen::Vector3d dcm_vel;

  Eigen::Vector3d com_vel_est;
  Eigen::Vector3d imu_ang_vel_est;
  Eigen::Vector3d cam_est;

  Eigen::Quaternion<double> nominal_base_quat;

  Eigen::Isometry3d nominal_lfoot_iso;
  Eigen::Isometry3d nominal_rfoot_iso;

  int b_rf_contact;
  int b_lf_contact;

  int planning_id = 0;

private:
  DracoStateProvider();
};
