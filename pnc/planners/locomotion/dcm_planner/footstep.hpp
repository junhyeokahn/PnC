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

  Eigen::Vector3d getPos() const { return position; }
  Eigen::Matrix3d getRotMat() const { return R_ori; }
  Eigen::Quaterniond getOrientation() const { return orientation; }

  void setPos(const Eigen::Vector3d pos) { position = pos; }
  void setOri(const Eigen::Quaterniond &ori) {
    orientation = ori;
    R_ori = ori.toRotationMatrix();
  }

  /// Print out this footstep information.
  void printInfo();

  // Footstep generation static method for utility function
  static std::vector<Footstep>
  getFwdWalkFootStep(const int n_steps, const double forward_step_length,
                     const double nominal_footwidth, const int first_swing_leg,
                     const Footstep &current_mid_foot);

  static std::vector<Footstep>
  getInPlaceWalkFootStep(const int n_steps, const double nominal_footwidth,
                         const int first_swing_leg,
                         const Footstep &current_mid_foot);

  static std::vector<Footstep>
  getTurningFootStep(const int n_steps, const double turn_radians_per_step,
                     const double nominal_footwidth,
                     const Footstep &current_mid_foot);

  static std::vector<Footstep>
  getStrafeFootStep(const int n_steps, const double strafe_distance,
                    const double nominal_footwidth,
                    const Footstep &current_mid_foot);

private:
  void common_initialization();
};
