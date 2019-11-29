#pragma once

#include <PnC/Controller.hpp>

class DracoStateProvider;
class RobotSystem;
class WBLC;
class WBLC_ExtraData;
class KinWBC;
class ContactSpec;
class CMPC;
class IHWBC;

class MPCBalanceCtrl : public Controller {
   public:
    MPCBalanceCtrl(RobotSystem*);
    virtual ~MPCBalanceCtrl();

    virtual void oneStep(void* _cmd);
    virtual void firstVisit();
    virtual void lastVisit();
    virtual bool endOfPhase();
    virtual void ctrlInitialization(const YAML::Node& node);

    void setStanceTime(double time) { end_time_ = time; }
    void setCoMHeight(double height) {
        target_com_height_ = height;
    }
    void SetStabilizationDuration(double time){
        stab_dur_ = time;
    }

   protected:
    Eigen::VectorXd Kp_, Kd_;
    Eigen::VectorXd des_jpos_;
    Eigen::VectorXd des_jvel_;
    Eigen::VectorXd des_jacc_;

    Eigen::VectorXd jpos_ini_;
    double end_time_;
    int dim_contact_;

    Task* com_task_;
    Task* total_joint_task_;

    Task* body_ori_task_;
    Task* rfoot_center_rz_xyz_task;
    Task* lfoot_center_rz_xyz_task;

    ContactSpec* rfoot_front_contact_;
    ContactSpec* rfoot_back_contact_;
    ContactSpec* lfoot_front_contact_;
    ContactSpec* lfoot_back_contact_;

    KinWBC* kin_wbc_;
    WBLC* wblc_;
    WBLC_ExtraData* wblc_data_;

    // Convex MPC
    CMPC* convex_mpc;
    double last_control_time_;

    // IHWBC
    IHWBC* ihwbc;

    double target_com_height_;
    double stab_dur_;
    Eigen::VectorXd ini_com_pos_;
    Eigen::VectorXd ini_com_vel_;
    Eigen::VectorXd goal_com_pos_;

    void task_setup();
    void contact_setup();

    void _compute_torque_wblc(Eigen::VectorXd& gamma);

    double ctrl_start_time_;
    DracoStateProvider* sp_;

    double des_com_offset_x_;

    // ihwbc
    Eigen::VectorXd q_current_;
    Eigen::VectorXd qdot_current_;

    Eigen::VectorXd w_task_heirarchy_;
    double w_contact_weight_;
    double lambda_qddot_;
    double lambda_Fr_;
    void _compute_torque_ihwbc(Eigen::VectorXd& gamma);

    double ihwbc_dt_;
    Eigen::VectorXd tau_cmd_;
    Eigen::VectorXd qddot_cmd_;
    Eigen::VectorXd qdot_des_;
    Eigen::VectorXd q_des_;


    // MPC Functions
    void _mpc_setup();
    void _mpc_Xdes_setup();
    void _mpc_solve();

    // MPC Variables
    double mpc_horizon_;
    double mpc_dt_; 

    Eigen::VectorXd mpc_x0_;
    Eigen::VectorXd mpc_Xdes_;
    Eigen::MatrixXd mpc_r_feet_;
    Eigen::VectorXd mpc_x_pred_;
    Eigen::VectorXd mpc_Fd_out_;

    Eigen::VectorXd mpc_Fd_des_;

    Eigen::Vector3d com_current_;
    Eigen::Vector3d com_rate_current_;
    Eigen::Vector3d midfeet_pos_;


};

