#pragma once

#include <PnC/Controller.hpp>

class DracoStateProvider;
class RobotSystem;
class ContactSpec;
class IHWBC;
class Task;

class WalkingReferenceTrajectoryModule;
class DracoFootstep;

class HermiteCurveVec;
class HermiteQuaternionCurve;

class SingleSupportCtrl : public Controller {
    public:
        SingleSupportCtrl(RobotSystem*, WalkingReferenceTrajectoryModule*);
        virtual ~SingleSupportCtrl();

        virtual void oneStep(void* _cmd);
        virtual void firstVisit();
        virtual void lastVisit();
        virtual bool endOfPhase();
        virtual void ctrlInitialization(const YAML::Node& node);

        void setTotalCtrlTime(double val) {end_time_ = val;}

        void setCoMHeight(double val) {target_com_height_ = val;}

    protected:
        WalkingReferenceTrajectoryModule* walking_reference_trajectory_module_;

        double end_time_;
        int dim_contact_;

        Eigen::Vector3d ini_com_pos_;
        Eigen::Vector3d ini_com_vel_;
        Eigen::Vector3d goal_com_pos_;
        double target_com_height_;

        Eigen::VectorXd tau_cmd_;
        Eigen::VectorXd tau_cmd_old_;
        Eigen::VectorXd qddot_cmd_;

        // Tasks
        Task* com_task_;
        Task* bodyori_task_;
        Task* rfoot_front_task_;
        Task* rfoot_back_task_;
        Task* lfoot_front_task_;
        Task* lfoot_back_task_;
        Task* rfoot_line_task_;
        Task* lfoot_line_task_;

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

        // Foot Trajectory
        std::shared_ptr<HermiteCurveVec> foot_pos_traj_init_to_mid_;
        std::shared_ptr<HermiteCurveVec> foot_pos_traj_mid_to_end_;
        std::shared_ptr<HermiteCurveVec> pos_traj_to_use;
        std::shared_ptr<HermiteQuaternionCurve> foot_ori_trajectory;

        void _compute_torque_ihwbc();
        void _task_setup();
        void _contact_setup();
        void _compute_swing_foot_trajectory();
};
