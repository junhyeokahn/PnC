#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

#include "configuration.hpp"

/// class Footstep
class Footstep {
public:
  /// \{ \name Constructor and Destructor
  Footstep();
  Footstep(const Eigen::Vector3d &pos_in, const Eigen::Quaterniond &quat_in,
           const int &robot_side_in);

  ~Footstep();
  /// \}

  /// Position of the footstep.
  Eigen::Vector3d position;
  /// Orientation of the footstep.
  Eigen::Quaternion<double> orientation;
  /// Orientation of the footstep.
  Eigen::Matrix3d R_ori;

  // Set position, orientation, side.
  void setPosOriSide(const Eigen::Vector3d &pos_in,
                     const Eigen::Quaterniond &quat_in,
                     const int &robot_side_in);
  // Set position, orientation.
  void setPosOri(const Eigen::Vector3d &pos_in,
                 const Eigen::Quaterniond &quat_in);
  // Set side.
  void setRightSide();
  // Set side.
  void setLeftSide();
  // Set side.
  void setMidFoot();

  /// Footstep side.
  int robot_side;

  /// Return the mid foot.
  void computeMidfeet(const Footstep &footstep1, const Footstep &footstep2,
                      Footstep &midfeet);

  /// Return footstep side.
  int getSide();

  /// Print out this footstep information.
  void printInfo();

private:
  void common_initialization();
};
