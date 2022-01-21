#pragma once

#include <Eigen/Dense>
#include <map>

class HumanoidSensorData {
public:
    HumanoidSensorData();
    ~HumanoidSensorData();

    Eigen::Matrix<double, 6, 1> imu_frame_vel;
    std::map<std::string, double> joint_positions;
    std::map<std::string, double> joint_velocities;

    bool b_rf_contact;
    bool b_lf_contact;

};