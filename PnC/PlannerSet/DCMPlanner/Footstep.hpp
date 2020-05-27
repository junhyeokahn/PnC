#ifndef FOOTSTEP_OBJECT_H
#define FOOTSTEP_OBJECT_H

#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <string>

#define LEFT_ROBOT_SIDE 0
#define RIGHT_ROBOT_SIDE 1
#define MIDFOOT_TYPE 2

// Data container for a footstep

class Footstep{
public:
  Footstep();
  Footstep(const Eigen::Vector3d & pos_in, const Eigen::Quaterniond & quat_in, const int & robot_side_in);

  ~Footstep(); 

  // Position and orientation of the sole of the footstep
  Eigen::Vector3d position;
  Eigen::Quaternion<double> orientation;
  Eigen::Matrix3d R_ori;

  // Setters
  void setPosOriSide(const Eigen::Vector3d & pos_in, const Eigen::Quaterniond & quat_in, const int & robot_side_in);
  void setPosOri(const Eigen::Vector3d & pos_in, const Eigen::Quaterniond & quat_in);
  void setRightSide();
  void setLeftSide();
  void setMidFoot();

  // Left or right side
  int robot_side;

  void computeMidfeet(const Footstep & footstep1, const Footstep & footstep2, Footstep & midfeet);

  int getSide();
  void printInfo();

private:
  void common_initialization();

};

#endif
