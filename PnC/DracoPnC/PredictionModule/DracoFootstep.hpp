#ifndef DRACO_FOOTSTEP_OBJECT_H
#define DRACO_FOOTSTEP_OBJECT_H

#include <Eigen/Dense>
#include <iostream>
#include <string>

#define DRACO_LEFT_FOOTSTEP 0
#define DRACO_RIGHT_FOOTSTEP 1
#define DRACO_MID_FOOTSTEP 2

// Data container for a footstep by Draco

class DracoFootstep{
public:
  DracoFootstep();
  DracoFootstep(const Eigen::Vector3d & pos_in, const Eigen::Quaterniond & quat_in, const int & robot_side_in);

  ~DracoFootstep(); 

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

  void setWalkingParams(const double double_contact_time_in,
                        const double contact_transition_time_in,
                        const double swing_time_in,
                        const double swing_height_in);

  // Left or right side
  int robot_side;

  void computeMidfeet(const DracoFootstep & footstep1, const DracoFootstep & footstep2, DracoFootstep & midfeet);

  int getSide();
  void printInfo();

  // distance of toe and heel contacts from the center of the foot.
  double toe_dist_from_center = 0.1;
  double heel_dist_from_center = 0.05;

  // default walking parameters
  double double_contact_time = 0.03; // seconds
  double contact_transition_time = 0.2; // seconds
  double swing_time = 0.3; // seconds
  double swing_height = 0.05; // meters

  // DCM walking parameters
  double t_init = 0.1; // initial transfer time
  double t_ds = 0.2; // double support transfer time
  double t_ss = 0.3; // single support time
  double alpha_ds = 0.5; // value between 0.0 and 1.0 for double support DCM interpolation

  std::vector<Eigen::Vector3d> local_contact_point_list;
  std::vector<Eigen::Vector3d> global_contact_point_list;

  Eigen::Vector3d getToePosition();
  Eigen::Vector3d getHeelPosition();


private:
  void common_initialization();
  void updateContactLocations();

};

#endif