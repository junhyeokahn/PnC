#pragma once

#include <Utils/Math/BSplineBasic.h>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <Utils/Math/minjerk_one_dim.hpp>

class KinWBC;

class BodyFootTurningCtrl : public SwingPlanningCtrl {
   public:
    BodyFootTurningCtrl(RobotSystem* robot, std::string swing_foot_,
                        FootStepPlanner* planner);
    virtual ~BodyFootTurningCtrl();
    virtual void oneStep(void* _cmd);
    virtual void firstVisit();
    virtual void lastVisit() { sp_->des_jpos_prev = des_jpos_; }
    virtual bool endOfPhase();

    virtual void ctrlInitialization(const YAML::Node& node);

   protected:
    double ini_base_height_;
    int swing_leg_jidx_;
    double push_down_height_;  // push foot below the ground at landing
    Eigen::Vector3d initial_target_loc_;

    int dim_contact_;
    // [right_front, right_back, left_front, left_back]
    std::vector<int> fz_idx_in_cost_;
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
    std::vector<ContactSpec*> kin_wbc_contact_list_;

    Task* base_task_;
    Task* foot_task_;
    Task* foot_ry_rz_task_;

    KinWBC* kin_wbc_;
    Eigen::VectorXd des_jpos_;
    Eigen::VectorXd des_jvel_;
    Eigen::VectorXd des_jacc_;

    Eigen::VectorXd Kp_;
    Eigen::VectorXd Kd_;

    Eigen::Vector3d ini_com_pos_;
    Eigen::Vector3d ini_foot_pos_;
    Eigen::Quaternion<double> ini_foot_ori_;
    Eigen::Quaternion<double> foot_delta_quat_;
    Eigen::Vector3d foot_delta_so3_;

    Eigen::VectorXd ini_config_;

    double fin_foot_z_vel_;
    double fin_foot_z_acc_;

    std::vector<double> foot_landing_offset_;

    Eigen::VectorXd body_pt_offset_;
    Eigen::VectorXd default_target_loc_;

    std::vector<MinJerk_OneDimension*> min_jerk_offset_;
    BS_Basic<3, 3, 1, 2, 2> foot_traj_;
};
