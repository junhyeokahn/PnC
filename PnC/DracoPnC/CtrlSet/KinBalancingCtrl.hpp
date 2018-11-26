#pragma once

#include <PnC/Controller.hpp>

class DracoStateProvider;
class RobotSystem;
class WBLC;
class WBLC_ExtraData;
class KinWBC;
class ContactSpec;

// TODO : test wbdc, line contact, surface contact, com task, centroid task
class KinBalancingCtrl: public Controller{
    public:
        KinBalancingCtrl(RobotSystem* );
        virtual ~KinBalancingCtrl();

        virtual void oneStep(void* _cmd);
        virtual void firstVisit();
        virtual void lastVisit();
        virtual bool endOfPhase();
        virtual void ctrlInitialization(const std::string & setting_file_name);

        void setBalancingTime(double time) { end_time_ = time; }
        void setInterpolationTime(double time) { interpolation_dur_ = time; }

    protected:
        Eigen::VectorXd Kp_, Kd_;
        Eigen::VectorXd des_jpos_;
        Eigen::VectorXd des_jvel_;
        Eigen::VectorXd des_jacc_;

        Eigen::VectorXd jpos_ini_;
        Eigen::Vector3d ini_com_pos_;
        Eigen::Vector3d goal_com_pos_;

        double interpolation_dur_;
        double end_time_;
        int dim_contact_;

        ContactSpec* rfoot_contact_;
        ContactSpec* lfoot_contact_;

        Task* body_rpz_task_;
        std::vector<int> selected_jidx_;
        Task* selected_joint_task_;

        KinWBC* kin_wbc_;
        WBLC* wblc_;
        WBLC_ExtraData* wblc_data_;

        void _task_setup();
        void _contact_setup();
        void _compute_torque(Eigen::VectorXd & gamma);

        double ctrl_start_time_;
        DracoStateProvider* sp_;
};
