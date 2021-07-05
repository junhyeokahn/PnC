#include <pnc/planners/locomotion/dcm_planner/footstep.hpp>

Footstep::Footstep() {
  position.setZero();
  orientation.setIdentity();
  robot_side = -1;
  common_initialization();
}

Footstep::Footstep(const Eigen::Vector3d &pos_in,
                   const Eigen::Quaterniond &quat_in,
                   const int &robot_side_in) {
  position = pos_in;
  orientation = quat_in;
  robot_side = robot_side_in;
  common_initialization();
}

Footstep::~Footstep() {}

void Footstep::setPosOriSide(const Eigen::Vector3d &pos_in,
                             const Eigen::Quaterniond &quat_in,
                             const int &robot_side_in) {
  position = pos_in;
  orientation = quat_in;
  robot_side = robot_side_in;
  R_ori = orientation.toRotationMatrix();
}

void Footstep::setPosOri(const Eigen::Vector3d &pos_in,
                         const Eigen::Quaterniond &quat_in) {
  position = pos_in;
  orientation = quat_in;
  R_ori = orientation.toRotationMatrix();
}

void Footstep::setRightSide() { robot_side = EndEffector::RFoot; }
void Footstep::setLeftSide() { robot_side = EndEffector::LFoot; }

void Footstep::common_initialization() {
  R_ori = orientation.toRotationMatrix();
}

void Footstep::printInfo() {
  if ((robot_side == EndEffector::LFoot) ||
      (robot_side == EndEffector::RFoot)) {
    std::cout << "side = "
              << (robot_side == EndEffector::LFoot ? "EndEffector::LFoot"
                                                   : "EndEffector::RFoot")
              << std::endl;
  } else if (robot_side == -1) {
    std::cout << "MIDFOOT_TYPE" << std::endl;
  }

  std::cout << "pos: " << position[0] << ", " << position[1] << ", "
            << position[2] << std::endl;
  std::cout << "ori: " << orientation.x() << ", " << orientation.y() << ", "
            << orientation.z() << ", " << orientation.w() << std::endl;
}

void Footstep::computeMidfeet(const Footstep &footstep1,
                              const Footstep &footstep2, Footstep &midfeet) {
  midfeet.position = 0.5 * (footstep1.position + footstep2.position);
  midfeet.orientation = footstep1.orientation.slerp(0.5, footstep2.orientation);
  midfeet.R_ori = midfeet.orientation.toRotationMatrix();
  midfeet.robot_side = -1;
}
