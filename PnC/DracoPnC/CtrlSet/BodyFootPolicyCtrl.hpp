#pragma once

#include <Utils/Math/BSplineBasic.h>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <ReinforcementLearning/RLInterface/NeuralNetModel.hpp>
#include <Utils/Math/minjerk_one_dim.hpp>

class KinWBC;

class BodyFootPolicyCtrl : public SwingPlanningCtrl {
   public:
    BodyFootPolicyCtrl(RobotSystem* robot, std::string swing_foot_,
                       FootStepPlanner* planner);
    virtual ~BodyFootPolicyCtrl();
    virtual void oneStep(void* _cmd);
    virtual void firstVisit();
    virtual void lastVisit() {
        sp_->des_jpos_prev = des_jpos_;
        // std::cout << "[BodyFootPlanning] End "<< std::endl;
    }
    virtual bool endOfPhase();

    virtual void ctrlInitialization(const YAML::Node& node);

    void setTerminateObsLowerBound(Eigen::VectorXd lb) {
        terminate_obs_lower_bound_ = lb;
    }
    void setTerminateObsUpperBound(Eigen::VectorXd ub) {
        terminate_obs_upper_bound_ = ub;
    }
    void setActionScale(Eigen::VectorXd as) { action_scale_ = as; }
    void setActionLowerBound(Eigen::VectorXd lb) {
        action_lower_bound_ = lb;
        action_lower_bound_mat_ = Eigen::MatrixXd::Zero(1, lb.size());
        for (int i = 0; i < lb.size(); ++i) {
            action_lower_bound_mat_(0, i) = lb(i);
        }
    }
    void setActionUpperBound(Eigen::VectorXd ub) {
        action_upper_bound_ = ub;
        action_upper_bound_mat_ = Eigen::MatrixXd::Zero(1, ub.size());
        for (int i = 0; i < ub.size(); ++i) {
            action_upper_bound_mat_(0, i) = ub(i);
        }
    }

   protected:
    // rl parameters
    Eigen::VectorXd terminate_obs_lower_bound_;
    Eigen::VectorXd terminate_obs_upper_bound_;
    Eigen::VectorXd action_scale_;
    Eigen::VectorXd action_lower_bound_;
    Eigen::VectorXd action_upper_bound_;
    Eigen::MatrixXd action_lower_bound_mat_;
    Eigen::MatrixXd action_upper_bound_mat_;
    NeuralNetModel* nn_policy_;
    NeuralNetModel* nn_valfn_;

    bool b_use_policy_;

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

    std::vector<int> selected_jidx_;
    Task* selected_joint_task_;
    Task* base_task_;
    // Task* foot_pitch_task_;
    Task* foot_point_task_;

    KinWBC* kin_wbc_;
    Eigen::VectorXd des_jpos_;
    Eigen::VectorXd des_jvel_;
    Eigen::VectorXd des_jacc_;

    Eigen::VectorXd Kp_;
    Eigen::VectorXd Kd_;

    Eigen::Vector3d ini_com_pos_;
    Eigen::Vector3d ini_foot_pos_;

    Eigen::VectorXd ini_config_;

    double ini_ankle_;
    double fin_ankle_;
    double switch_vel_threshold_;
    double fin_foot_z_vel_;
    double fin_foot_z_acc_;

    std::vector<double> foot_landing_offset_;

    Eigen::VectorXd body_pt_offset_;
    Eigen::VectorXd default_target_loc_;

    std::vector<MinJerk_OneDimension*> min_jerk_offset_;
    BS_Basic<3, 3, 1, 2, 2> foot_traj_;
};
