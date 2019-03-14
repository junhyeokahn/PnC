#pragma once

#include <Configuration.h>
#include <Utils/IO/IOUtilities.hpp>

class RobotSystem;

class AtlasStateProvider {
   public:
    static AtlasStateProvider* getStateProvider(RobotSystem* _robot);
    ~AtlasStateProvider() {}

    int stance_foot;
    double curr_time;

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;

    Eigen::Vector3d global_pos_local;
    Eigen::VectorXd des_jpos_prev;
    Eigen::VectorXd jpos_ini;

    Eigen::Vector2d des_location;
    Eigen::Quaternion<double> des_quat;
    Eigen::Vector2d walking_velocity;

    int b_rfoot_contact;
    int b_lfoot_contact;

    Eigen::Vector3d rfoot_pos;
    Eigen::Vector3d lfoot_pos;
    Eigen::Quaternion<double> rfoot_quat;
    Eigen::Quaternion<double> lfoot_quat;
    Eigen::Vector3d rfoot_vel;
    Eigen::Vector3d lfoot_vel;
    Eigen::Vector3d rfoot_so3;
    Eigen::Vector3d lfoot_so3;

    int num_step_copy;
    int phase_copy;

    void saveCurrentData();

   private:
    AtlasStateProvider(RobotSystem* _robot);
    RobotSystem* robot_;
};
