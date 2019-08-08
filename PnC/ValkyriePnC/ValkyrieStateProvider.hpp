#pragma once
#include <utility>

#include <Configuration.h>
#include <Utils/General/Clock.hpp>
#include <Utils/IO/IOUtilities.hpp>

class RobotSystem;

class ValkyrieStateProvider {
   public:
    static ValkyrieStateProvider* getStateProvider(RobotSystem* _robot);
    ~ValkyrieStateProvider() {}

    void saveCurrentData();

    Clock clock;

    int stance_foot;
    double curr_time;

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

   private:
    ValkyrieStateProvider(RobotSystem* _robot);
    RobotSystem* robot_;
};
