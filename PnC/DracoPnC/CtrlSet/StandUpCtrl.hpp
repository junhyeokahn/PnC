#pragma once

#include <PnC/Controller.hpp>

class DracoStateProvider;
class RobotSystem;
class ContactSpec;
class IHWBC;
class Task;

class StandUpCtrl : public Controller {
    public:
        StandUpCtrl(RobotSystem*);
        virtual ~StandUpCtrl();

        virtual void oneStep(void* _cmd);
        virtual void firstVisit();
        virtual void lastVisit();
        virtual bool endOfPhase();
        virtual void ctrlInitialization(const YAML::Node& node);

        void setCoMHeight(double height) {
            target_com_height_ = height;
        }

        void setTotalCtrlTime(double dur) {
            end_time_ = dur;
        }

    protected:

        double end_time_;
        double target_com_height_;
        double max_fz_;
        int dim_contact_;
        Eigen::Vector3d ini_com_pos_;
        Eigen::Vector3d target_com_pos_;
        double ctrl_start_time_;

        Eigen::VectorXd tau_cmd_;
        Eigen::VectorXd tau_cmd_old_;
        Eigen::VectorXd qddot_cmd_;
        Eigen::VectorXd qdot_des_;
        Eigen::VectorXd q_des_;

        // Tasks
        Task* com_task_;
        Task* bodyori_task_;
        Task* rfoot_front_task_;
        Task* rfoot_back_task_;
        Task* lfoot_front_task_;
        Task* lfoot_back_task_;

        // Contacts
        ContactSpec* rfoot_front_contact_;
        ContactSpec* rfoot_back_contact_;
        ContactSpec* lfoot_front_contact_;
        ContactSpec* lfoot_back_contact_;

        // IHWBC
        double com_task_weight_;
        double bodyori_task_weight_;
        double rfoot_task_weight_;
        double lfoot_task_weight_;
        Eigen::VectorXd task_weight_heirarchy_;
        double rf_tracking_weight_;
        double qddot_reg_weight_;
        double rf_reg_weight_;
        IHWBC* ihwbc_;

        double velocity_break_freq_;
        double position_break_freq_;
        double max_jvel_;
        double max_jpos_error_;

        void _compute_torque_ihwbc();
        void _task_setup();
        void _contact_setup();

        DracoStateProvider* sp_;
};
