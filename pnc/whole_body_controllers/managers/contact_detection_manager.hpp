#pragma once

#include "pnc/robot_system/robot_system.hpp"
#include "utils/util.hpp"

#include <iterator>
#include <map>

class ContactDetectionManager {
public:
    ContactDetectionManager(RobotSystem *_robot, std::string lfoot_id_name,
                            std::string rfoot_id_name);

    void update_contact_stance(int &swing_side);
    bool check_swing_foot_contact(double &expected_height_difference);

private:
    RobotSystem *robot_;

    std::map<int, Eigen::Vector2d> foot_corner_map_;
    std::string swing_leg_name_;
    std::string support_leg_name_;
    double contact_tol_;

};


