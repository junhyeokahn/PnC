#pragma once
#include <map>
#include <string>

#include <Configuration.h>
#include <Utils/IO/IOUtilities.hpp>

class RobotSystem;

class AtlasStateProvider {
public:
  static AtlasStateProvider *getStateProvider(RobotSystem *_robot);
  ~AtlasStateProvider() {}

  std::map<std::string, double> nominal_joint_pos;

  int state;
  int prev_state;

  double curr_time;
  double dt = 0.001;

  Eigen::Vector3d dcm;
  Eigen::Vector3d prev_dcm;
  Eigen::Vector3d dcm_vel;

  int b_rf_contact;
  int b_lf_contact;

private:
  AtlasStateProvider(RobotSystem *_robot);
  RobotSystem *robot_;
};
