#pragma once
#include <utility>

#include <Configuration.h>
#include <PnC/RobotSystem/CentroidModel.hpp>
#include <Utils/General/Clock.hpp>
#include <Utils/IO/IOUtilities.hpp>

class RobotSystem;

class ValkyrieStateProvider {
   public:
    static ValkyrieStateProvider* getStateProvider(RobotSystem* _robot);
    ~ValkyrieStateProvider() {}

    void saveCurrentData();

    Clock clock;

    double curr_time;
    double prev_state_machine_time;
    double planning_moment;

    int stance_foot;
    Eigen::Isometry3d stance_foot_iso;
    Eigen::Isometry3d moving_foot_target_iso;

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;

    Eigen::VectorXd jpos_ini;

    int b_rfoot_contact;
    int b_lfoot_contact;

    int num_step_copy;
    int phase_copy;

    // save planned result for the plot
    std::vector<Eigen::Isometry3d> foot_target_list;
    std::vector<Eigen::VectorXd> com_des_list;

    // API related variable
    bool b_walking;
    double ft_length;
    double r_ft_width;
    double l_ft_width;
    double ft_ori_inc;
    int num_total_step;
    int num_residual_step;

    // data manager
    Eigen::VectorXd com_pos;
    Eigen::VectorXd com_vel;
    Eigen::VectorXd mom;

    Eigen::VectorXd com_pos_des;
    Eigen::VectorXd com_vel_des;
    Eigen::VectorXd mom_des;

    Eigen::VectorXd rf_pos;
    Eigen::VectorXd rf_vel;
    Eigen::VectorXd lf_pos;
    Eigen::VectorXd lf_vel;

    Eigen::VectorXd rf_pos_des;
    Eigen::VectorXd rf_vel_des;
    Eigen::VectorXd lf_pos_des;
    Eigen::VectorXd lf_vel_des;

    Eigen::Quaternion<double> rf_ori_quat;
    Eigen::VectorXd rf_ang_vel;
    Eigen::Quaternion<double> lf_ori_quat;
    Eigen::VectorXd lf_ang_vel;

    Eigen::Quaternion<double> rf_ori_quat_des;
    Eigen::VectorXd rf_ang_vel_des;
    Eigen::Quaternion<double> lf_ori_quat_des;
    Eigen::VectorXd lf_ang_vel_des;

    Eigen::Quaternion<double> pelvis_ori;
    Eigen::VectorXd pelvis_ang_vel;
    Eigen::Quaternion<double> torso_ori;
    Eigen::VectorXd torso_ang_vel;

    Eigen::Quaternion<double> pelvis_ori_des;
    Eigen::VectorXd pelvis_ang_vel_des;
    Eigen::Quaternion<double> torso_ori_des;
    Eigen::VectorXd torso_ang_vel_des;

    Eigen::VectorXd r_rf_des;
    Eigen::VectorXd l_rf_des;
    Eigen::VectorXd r_rf;
    Eigen::VectorXd l_rf;

    Eigen::VectorXd des_jacc_cmd;

   private:
    ValkyrieStateProvider(RobotSystem* _robot);
    RobotSystem* robot_;
};
