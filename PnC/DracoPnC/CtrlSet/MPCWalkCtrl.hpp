#pragma once

#include <PnC/Controller.hpp>

class DracoStateProvider;
class RobotSystem;
class ContactSpec;
class CMPC;
class IHWBC;
class MPCDesiredTrajectoryManager;
class GaitCycle;

class WalkingReferenceTrajectoryModule;
class DracoFootstep;

class MPCWalkCtrl : public Controller {
   public:
    MPCWalkCtrl(RobotSystem*);
    virtual ~MPCWalkCtrl();

    virtual void oneStep(void* _cmd);
    virtual void firstVisit();
    virtual void lastVisit();
    virtual bool endOfPhase();
    virtual void ctrlInitialization(const YAML::Node& node);

    void setStanceTime(double time) { end_time_ = time; }
    void setCoMHeight(double height) {
        target_com_height_ = height;
    }

    void setContactTransitionTime(double time){
        contact_transition_dur_ = time;
    }

    void setWalkStartTime(double start_time){
        walk_start_time_ = start_time;
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
    Eigen::VectorXd jdot_ini_;

    double end_time_;
    int dim_contact_;

    Task* com_task_;
    Task* total_joint_task_;

    Task* body_ori_task_;
    Task* rfoot_center_rz_xyz_task;
    Task* lfoot_center_rz_xyz_task;

    Task* rfoot_front_task;
    Task* rfoot_back_task;
    Task* lfoot_front_task;
    Task* lfoot_back_task;

    Task* ang_momentum_task;

    ContactSpec* rfoot_front_contact_;
    ContactSpec* rfoot_back_contact_;
    ContactSpec* lfoot_front_contact_;
    ContactSpec* lfoot_back_contact_;

    // Footstep list
    std::vector<DracoFootstep> desired_footstep_list_;

    // Sway Behavior
    double walk_start_time_;
    double sway_magnitude_;
    double sway_period_;

    // Convex MPC
    CMPC* convex_mpc;
    double last_control_time_;

    bool simulate_mpc_solved_;

    std::shared_ptr<GaitCycle> no_gait_;
    std::shared_ptr<GaitCycle> biped_gait_;
    // Set custom gait cycle
    double gait_swing_time_;
    double gait_transition_time_;
    double gait_biped_walking_offset_;
    double gait_total_gait_duration_;

    bool gait_set_once_;

    // Desired trajectory Managers
    WalkingReferenceTrajectoryModule* reference_trajectory_module_;
    DracoFootstep* left_foot_start_;
    DracoFootstep* right_foot_start_;

    bool references_set_once_;
    void references_setup();

    //  internal state machines 
    int ctrl_state_;
    int prev_ctrl_state_; 


    MPCDesiredTrajectoryManager* mpc_old_trajectory_;
    MPCDesiredTrajectoryManager* mpc_new_trajectory_;

    double homotopy_merge_time_;


    // IHWBC
    IHWBC* ihwbc;
    Eigen::VectorXd gamma_old_;

    double target_com_height_;
    double contact_transition_dur_;
    double stab_dur_;
    Eigen::VectorXd ini_com_pos_;
    Eigen::VectorXd ini_com_vel_;
    Eigen::VectorXd goal_com_pos_;

    void task_setup();
    void contact_setup();

    double ctrl_start_time_;
    DracoStateProvider* sp_;

    // ihwbc
    Eigen::VectorXd q_current_;
    Eigen::VectorXd qdot_current_;

    Eigen::VectorXd w_task_heirarchy_;
    double w_task_rfoot_;
    double w_task_lfoot_;
    double w_task_com_;
    double w_task_body_;
    double w_task_joint_;

    double w_task_ang_momentum_;

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

    void _updateTrajectories();

    double mpc_t_start_solve_; // starting time for the mpc to solve
    bool mpc_solved_once_;


    // MPC Variables
    double mpc_horizon_;
    double mpc_dt_; 
    double mpc_mu_;
    double mpc_max_fz_;
    double mpc_control_alpha_;
    double mpc_delta_smooth_;
    double mpc_toe_heel_smooth_;

    bool mpc_smooth_from_prev_;
    bool mpc_use_approx_inertia_;
    bool mpc_do_toe_heel_smoothing_;

    Eigen::VectorXd mpc_approx_inertia_input_;

    Eigen::VectorXd mpc_cost_vec_;
    Eigen::VectorXd mpc_x0_;
    Eigen::VectorXd mpc_Xdes_;
    Eigen::MatrixXd mpc_r_feet_;
    Eigen::VectorXd mpc_x_pred_;
    Eigen::VectorXd mpc_Fd_out_;

    Eigen::VectorXd mpc_Fd_des_;
    Eigen::VectorXd mpc_Fd_des_filtered_;
    double alpha_fd_;

    Eigen::Vector3d com_current_;
    Eigen::Vector3d com_rate_current_;
    Eigen::Vector3d midfeet_pos_;

    // Integration Parameters
    double max_joint_vel_;
    double velocity_break_freq_;

    double max_jpos_error_;
    double position_break_freq_;

    // clamping function helper
    double clamp_value(double in, double min, double max);

    // compute alpha from break frequency
    double computeAlphaGivenBreakFrequency(double hz, double dt);


};

