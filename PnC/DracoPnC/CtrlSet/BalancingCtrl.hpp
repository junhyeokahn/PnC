#pragma once

#include <PnC/Controller.hpp>
#include "Utils/BSplineBasic.h"

class DracoStateProvider;
class RobotSystem;
class WBDC;
class WBDC_ExtraData;
class ContactSpec;

// TODO : test wbdc, line contact, surface contact, com task, centroid task
class BalancingCtrl: public Controller{
    public:
        BalancingCtrl(RobotSystem* );
        virtual ~BalancingCtrl();

        virtual void oneStep(void* _cmd);
        virtual void firstVisit();
        virtual void lastVisit();
        virtual bool endOfPhase();
        virtual void ctrlInitialization(const std::string & setting_file_name);

        void setBalancingTime(double time) { end_time_ = time; }
        void setInterpolationTime(double time) { interpolation_dur_ = time; }

    protected:
        Eigen::VectorXd Kp_, Kd_;

        double interpolation_dur_;
        double end_time_;

        ContactSpec* rfoot_contact_;
        ContactSpec* lfoot_contact_;

        Task* centroid_task_;
        Eigen::VectorXd centroid_pos_des_;
        Eigen::VectorXd centroid_vel_des_;
        Eigen::VectorXd centroid_acc_des_;
        Eigen::VectorXd centroid_pos_act_;
        Eigen::VectorXd centroid_vel_act_;
        Eigen::VectorXd centroid_acc_act_;

        WBDC* wbdc_;
        WBDC_ExtraData* wbdc_data_;

        void _task_setup();
        void _contact_setup();
        void _compute_torque(Eigen::VectorXd & gamma);

        double ctrl_start_time_;

        Eigen::VectorXd ini_com_pos_;
        Eigen::VectorXd ini_com_vel_;
        Eigen::VectorXd goal_com_pos_;
        Eigen::VectorXd goal_com_vel_;
        BS_Basic<3, 3, 0, 2, 2> spline_;

        DracoStateProvider* sp_;
};
