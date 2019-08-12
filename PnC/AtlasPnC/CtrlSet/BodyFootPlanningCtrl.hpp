#pragma once

#include <Utils/Math/BSplineBasic.h>
#include <PnC/AtlasPnC/CtrlSet/SwingPlanningCtrl.hpp>
#include <PnC/AtlasPnC/TaskSet/TaskSet.hpp>
#include <Utils/Math/minjerk_one_dim.hpp>

class BodyFootPlanningCtrl : public SwingPlanningCtrl {
   public:
    BodyFootPlanningCtrl(RobotSystem* robot, int swing_foot,
                         TVRPlanner* planner);
    virtual ~BodyFootPlanningCtrl();
    virtual void oneStep(void* _cmd);
    virtual void firstVisit();
    virtual void lastVisit() { sp_->des_jpos_prev = des_jpos_; }
    virtual bool endOfPhase();
    virtual void ctrlInitialization(const YAML::Node& node);

   protected:
    double waiting_time_limit_;
    double ini_base_height_;
    int swing_leg_jidx_;
    double push_down_height_;  // push foot below the ground at landing

    Eigen::Vector3d default_target_loc_;
    Eigen::Vector3d initial_target_loc_;

    int dim_contact_;
    ContactSpec* rfoot_contact_;
    ContactSpec* lfoot_contact_;

    void _CheckPlanning();
    void _Replanning(Eigen::Vector3d& target_loc);
    void _contact_setup();
    void _task_setup();
    void _compute_torque_wblc(Eigen::VectorXd& gamma);
    void _SetMinJerkOffset(const Eigen::Vector3d& offset);
    void _SetBspline(const Eigen::Vector3d& st_pos,
                     const Eigen::Vector3d& des_pos);

    void _GetSinusoidalSwingTrajectory();
    void _GetBsplineSwingTrajectory();
    void _foot_pos_task_setup();
    std::vector<ContactSpec*> kin_wbc_contact_list_;

    Task* body_pos_task_;
    Task* body_ori_task_;
    Task* torso_ori_task_;
    Task* foot_pos_task_;
    Task* foot_ori_task_;
    Task* total_joint_task_;

    Eigen::VectorXd des_jpos_;
    Eigen::VectorXd des_jvel_;
    Eigen::VectorXd des_jacc_;

    Eigen::VectorXd Kp_;
    Eigen::VectorXd Kd_;

    // Task specification
    Eigen::VectorXd ini_body_pos_;
    Eigen::Vector3d ini_foot_pos_;
    Eigen::Vector2d body_pt_offset_;

    Eigen::Quaternion<double> ini_body_quat_;
    Eigen::Quaternion<double> body_delta_quat_;
    Eigen::Vector3d body_delta_so3_;

    Eigen::Quaternion<double> ini_torso_quat_;
    Eigen::Quaternion<double> torso_delta_quat_;
    Eigen::Vector3d torso_delta_so3_;

    Eigen::Quaternion<double> ini_foot_quat_;
    Eigen::Quaternion<double> foot_delta_quat_;
    Eigen::Vector3d foot_delta_so3_;

    std::vector<double> foot_landing_offset_;

    std::vector<MinJerk_OneDimension*> min_jerk_offset_;
    BS_Basic<3, 3, 1, 2, 2> foot_traj_;
};
