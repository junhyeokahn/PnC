#pragma once

#include <Configuration.h>
#include <Utils/IO/IOUtilities.hpp>

class RobotSystem;

class DracoStateProvider {
   public:
    static DracoStateProvider* getStateProvider(RobotSystem* _robot);
    ~DracoStateProvider() {}

    std::string stance_foot;
    double curr_time;
    int rl_count;
    double target_yaw;
    Eigen::Vector3d adjusted_foot;
    Eigen::Vector3d guided_foot;

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd rotor_inertia;

    Eigen::Vector3d global_pos_local;
    Eigen::Vector2d des_location;
    Eigen::Vector3d est_mocap_body_pos;
    Eigen::Vector2d est_mocap_body_vel;
    Eigen::VectorXd des_jpos_prev;

    int b_rfoot_contact;
    int b_lfoot_contact;

    Eigen::VectorXd qddot_cmd;
    Eigen::VectorXd reaction_forces;

    Eigen::Vector3d rfoot_contact_center_pos;
    Eigen::Vector3d lfoot_contact_center_pos;
    Eigen::Vector3d rfoot_contact_center_vel;
    Eigen::Vector3d lfoot_contact_center_vel;

    Eigen::VectorXd led_kin_data;

    Eigen::Vector3d com_pos;
    Eigen::Vector3d com_vel;
    Eigen::Vector3d est_com_vel;

    int num_step_copy;
    int phase_copy;

    double first_LED_x;
    double first_LED_y;

    void saveCurrentData();

   private:
    DracoStateProvider(RobotSystem* _robot);
    RobotSystem* robot_;
};
