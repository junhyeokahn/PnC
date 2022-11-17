#include "contact_detection_manager.hpp"

ContactDetectionManager::ContactDetectionManager(RobotSystem *_robot,
                               std::string _lfoot_idx, std::string _rfoot_idx) {
  util::PrettyConstructor(2, "ContactDetectionManager");

  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");

  auto foot_half_width = util::ReadParameter<double>(cfg["contact_estimator"], "foot_half_width");
  auto foot_half_length = util::ReadParameter<double>(cfg["contact_estimator"], "foot_half_length");
  contact_tol_ = util::ReadParameter<double>(cfg["contact_estimator"], "contact_tol");

  // set up feet corner point local positions w.r.t. ankle
  Eigen::Vector2d l_front_pos = Eigen::Vector2d(foot_half_width, foot_half_length);
  Eigen::Vector2d r_front_pos = Eigen::Vector2d(-foot_half_width, foot_half_length);
  Eigen::Vector2d r_back_pos = Eigen::Vector2d(-foot_half_width, -foot_half_length);
  Eigen::Vector2d l_back_pos = Eigen::Vector2d(foot_half_width, -foot_half_length);
  foot_corner_map_.insert(std::pair<int, Eigen::Vector2d>(0, l_front_pos));
  foot_corner_map_.insert(std::pair<int, Eigen::Vector2d>(1, r_front_pos));
  foot_corner_map_.insert(std::pair<int, Eigen::Vector2d>(2, r_back_pos));
  foot_corner_map_.insert(std::pair<int, Eigen::Vector2d>(3, l_back_pos));

  robot_ = _robot;
  support_leg_name_ = _lfoot_idx;
  swing_leg_name_ = _rfoot_idx;
}

void ContactDetectionManager::update_contact_stance(int &swing_side) {

  if (swing_side == EndEffector::LFoot) {
    swing_leg_name_ = "l_foot_contact";
    support_leg_name_ = "r_foot_contact";
  } else {
    swing_leg_name_ = "r_foot_contact";
    support_leg_name_ = "l_foot_contact";
  }

}

bool ContactDetectionManager::check_swing_foot_contact(double &expected_height_difference) {


  // get swing and support position and orientation
  Eigen::Vector3d swing_foot_pos = robot_->get_link_iso(swing_leg_name_).translation();
  Eigen::Matrix3d swing_foot_ori = robot_->get_link_iso(swing_leg_name_).linear();
  Eigen::Vector3d support_foot_pos = robot_->get_link_iso(support_leg_name_).translation();
  Eigen::Matrix3d support_foot_ori = robot_->get_link_iso(support_leg_name_).linear();

  double ankle_to_corner_height = 0.;
  double swing_foot_corner_height = 0.;
  bool foot_has_touchdown = false;
  for(auto const &[num, pos] : foot_corner_map_) {
    ankle_to_corner_height = swing_foot_ori(2,0) * pos(0) + swing_foot_ori(2,1) * pos(1);
    swing_foot_corner_height = swing_foot_pos.z() + ankle_to_corner_height;
    foot_has_touchdown = (swing_foot_corner_height - support_foot_pos(2)) < contact_tol_;

    if(foot_has_touchdown) {
      return true;
    }
  }

  return false;
}