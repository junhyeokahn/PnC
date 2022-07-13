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

std::vector<Footstep> Footstep::getFwdWalkFootStep(
    const int n_steps, const double forward_step_length,
    const double nominal_footwidth, const int first_swing_leg,
    const Footstep &current_mid_foot) {
  std::vector<Footstep> foot_step_list;

  Footstep new_foot_step;

  int swing_leg_side = first_swing_leg;
  for (int i(0); i < n_steps; ++i) {
    if (swing_leg_side == EndEffector::LFoot) {
      Eigen::Vector3d local_offset(static_cast<double>(i + 1) *
                                       forward_step_length,
                                   nominal_footwidth / 2., 0.);
      new_foot_step.setPosOriSide(
          current_mid_foot.getPos() +
              current_mid_foot.getRotMat() * local_offset,
          current_mid_foot.getOrientation(), EndEffector::LFoot);
      swing_leg_side = EndEffector::RFoot;
    } else {
      Eigen::Vector3d local_offset(static_cast<double>(i + 1) *
                                       forward_step_length,
                                   -nominal_footwidth / 2., 0.);
      new_foot_step.setPosOriSide(
          current_mid_foot.getPos() +
              current_mid_foot.getRotMat() * local_offset,
          current_mid_foot.getOrientation(), EndEffector::RFoot);
      swing_leg_side = EndEffector::LFoot;
    }
    foot_step_list.push_back(new_foot_step);
  }

  // add additional step forward to square the feet
  if (swing_leg_side == EndEffector::LFoot) {
    Eigen::Vector3d local_offset(static_cast<double>(n_steps) *
                                     forward_step_length,
                                 nominal_footwidth / 2., 0.);
    new_foot_step.setPosOriSide(
        current_mid_foot.getPos() + current_mid_foot.getRotMat() * local_offset,
        current_mid_foot.getOrientation(), EndEffector::LFoot);
  } else {
    Eigen::Vector3d local_offset(static_cast<double>(n_steps) *
                                     forward_step_length,
                                 -nominal_footwidth / 2., 0.);
    new_foot_step.setPosOriSide(
        current_mid_foot.getPos() + current_mid_foot.getRotMat() * local_offset,
        current_mid_foot.getOrientation(), EndEffector::RFoot);
  }
  foot_step_list.push_back(new_foot_step);

  return foot_step_list;
}

std::vector<Footstep> Footstep::getInPlaceWalkFootStep(
    const int n_steps, const double nominal_footwidth,
    const int first_swing_leg, const Footstep &current_mid_foot) {
  std::vector<Footstep> foot_step_list;

  Footstep new_foot_step;

  int swing_leg_side = first_swing_leg;
  for (int i(0); i < n_steps; ++i) {
    if (swing_leg_side == EndEffector::LFoot) {
      Eigen::Vector3d local_offset(0., nominal_footwidth / 2., 0.);
      new_foot_step.setPosOriSide(
          current_mid_foot.getPos() +
              current_mid_foot.getRotMat() * local_offset,
          current_mid_foot.getOrientation(), EndEffector::LFoot);
      swing_leg_side = EndEffector::RFoot;
    } else {
      Eigen::Vector3d local_offset(0., -nominal_footwidth / 2., 0.);
      new_foot_step.setPosOriSide(
          current_mid_foot.getPos() +
              current_mid_foot.getRotMat() * local_offset,
          current_mid_foot.getOrientation(), EndEffector::RFoot);
      swing_leg_side = EndEffector::LFoot;
    }
    foot_step_list.push_back(new_foot_step);
  }
  return foot_step_list;
}

std::vector<Footstep> Footstep::getTurningFootStep(
    const int n_steps, const double turn_radians_per_step,
    const double nominal_footwidth, const Footstep &current_mid_foot) {
  std::vector<Footstep> foot_step_list;

  Footstep new_right_foot, new_left_foot;
  Footstep rotated_mid_foot = current_mid_foot;

  Eigen::Quaterniond quat_increment_local(
      Eigen::AngleAxisd(turn_radians_per_step, Eigen::Vector3d::UnitZ()));
  Eigen::Vector3d local_offset(0., nominal_footwidth / 2., 0);
  for (int i(0); i < n_steps; ++i) {
    rotated_mid_foot.setOri(quat_increment_local *
                            rotated_mid_foot.getOrientation());

    new_left_foot.setPosOriSide(
        rotated_mid_foot.getPos() + rotated_mid_foot.getRotMat() * local_offset,
        rotated_mid_foot.getOrientation(), EndEffector::LFoot);
    new_right_foot.setPosOriSide(
        rotated_mid_foot.getPos() +
            rotated_mid_foot.getRotMat() * -local_offset,
        rotated_mid_foot.getOrientation(), EndEffector::RFoot);
    if (turn_radians_per_step > 0) {
      foot_step_list.push_back(new_left_foot);
      foot_step_list.push_back(new_right_foot);
    } else {
      foot_step_list.push_back(new_right_foot);
      foot_step_list.push_back(new_left_foot);
    }
  }
  return foot_step_list;
}

std::vector<Footstep>
Footstep::getStrafeFootStep(const int n_steps, const double strafe_distance,
                            const double nominal_footwidth,
                            const Footstep &current_mid_foot) {
  std::vector<Footstep> foot_step_list;

  Footstep new_right_foot, new_left_foot;
  Footstep strafed_mid_foot = current_mid_foot;

  Eigen::Vector3d strafe_vec(0., strafe_distance, 0.);
  Eigen::Vector3d local_offset(0., nominal_footwidth / 2., 0.);
  for (int i(0); i < n_steps; ++i) {
    strafed_mid_foot.setPos(strafed_mid_foot.getPos() +
                            strafed_mid_foot.getRotMat() * strafe_vec);

    new_left_foot.setPosOriSide(
        strafed_mid_foot.getPos() + strafed_mid_foot.getRotMat() * local_offset,
        strafed_mid_foot.getOrientation(), EndEffector::LFoot);
    new_right_foot.setPosOriSide(
        strafed_mid_foot.getPos() +
            strafed_mid_foot.getRotMat() * -local_offset,
        strafed_mid_foot.getOrientation(), EndEffector::RFoot);

    if (strafe_distance > 0) {
      foot_step_list.push_back(new_left_foot);
      foot_step_list.push_back(new_right_foot);
    } else {
      foot_step_list.push_back(new_right_foot);
      foot_step_list.push_back(new_left_foot);
    }
  }
  return foot_step_list;
}
