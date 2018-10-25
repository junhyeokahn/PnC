#pragma once

#include <PnC/Controller.hpp>

class DracoStateProvider;
class RobotSystem;
class WBLC;
class WBLC_ExtraData;
class KinWBC;
class ContactSpec;

class BodyCtrl: public Controller{
    public:
        BodyCtrl(RobotSystem* );
        virtual ~BodyCtrl();

        virtual void oneStep(void* _cmd);
        virtual void firstVisit();
        virtual void lastVisit();
        virtual bool endOfPhase();
        virtual void ctrlInitialization(const std::string & setting_file_name);

        void setStanceTime(double time) { end_time_ = time; }
        void setStanceHeight(double height){
            target_body_height_ = height;
            b_set_height_target_ = true;
         }

    protected:
        Eigen::VectorXd Kp_, Kd_;
        Eigen::VectorXd des_jpos_;
        Eigen::VectorXd des_jvel_;
        Eigen::VectorXd des_jacc_;

        Eigen::VectorXd jpos_ini_;
        bool b_set_height_target_;
        int trj_type_;
        double end_time_;
        int dim_contact_;

        std::vector<int> selected_jidx_;
        Task* body_rpz_task_;
        Task* selected_joint_task_;
        KinWBC* kin_wbc_;
        ContactSpec* rfoot_contact_;
        ContactSpec* lfoot_contact_;
        WBLC* wblc_;
        WBLC_ExtraData* wblc_data_;

        double target_body_height_;
        double ini_body_height_;
        Eigen::Vector3d ini_body_pos_;

        void _body_task_setup();
        void _double_contact_setup();
        void _compute_torque_wblc(Eigen::VectorXd & gamma);

        double ctrl_start_time_;
        DracoStateProvider* sp_;
};
