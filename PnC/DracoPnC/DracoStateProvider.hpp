#pragma once

#include <Configuration.h>
#include <Utils/IO/IOUtilities.hpp>

class RobotSystem;

class DracoStateProvider {
   public:
    static DracoStateProvider* getStateProvider(RobotSystem* _robot);
    ~DracoStateProvider() {}

    std::string stance_foot;
    std::string prev_stance_foot;

    bool b_walking;
    int num_total_steps;
    int num_residual_steps;

    double curr_time;

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
    Eigen::VectorXd filtered_rf;

    Eigen::Vector3d rfoot_center_pos;
    Eigen::Vector3d lfoot_center_pos;
    Eigen::Vector3d rfoot_center_vel;
    Eigen::Vector3d lfoot_center_vel;
    Eigen::Vector3d rfoot_center_so3;
    Eigen::Vector3d lfoot_center_so3;
    Eigen::Quaternion<double> rfoot_center_quat;
    Eigen::Quaternion<double> lfoot_center_quat;

    Eigen::VectorXd led_kin_data;

    Eigen::Vector3d com_pos;
    Eigen::Vector3d com_vel;
    Eigen::Vector3d est_com_vel;

    Eigen::Vector3d mpc_pred_pos;
    Eigen::Vector3d mpc_pred_vel;

    Eigen::Vector3d com_pos_des;
    Eigen::Vector3d com_vel_des;

    Eigen::VectorXd des_jpos;
    Eigen::VectorXd des_jvel;

    Eigen::Vector3d dcm;
    Eigen::Vector3d dcm_des;
    Eigen::Vector3d prev_dcm;
    Eigen::Vector3d dcm_vel;    
    Eigen::Vector3d dcm_vel_des;
    Eigen::Vector3d r_vrp;
    Eigen::Vector3d r_vrp_des;

    double omega;

    int num_step_copy;
    int phase_copy;

    double first_LED_x;
    double first_LED_y;

    void saveCurrentData();

   private:
    DracoStateProvider(RobotSystem* _robot);
    RobotSystem* robot_;
};
