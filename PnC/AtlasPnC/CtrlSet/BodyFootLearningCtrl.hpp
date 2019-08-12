#pragma once

#include <Utils/Math/BSplineBasic.h>
#include <PnC/AtlasPnC/CtrlSet/SwingPlanningCtrl.hpp>
#include <PnC/AtlasPnC/TaskSet/TaskSet.hpp>
#include <ReinforcementLearning/RLInterface/NeuralNetModel.hpp>
#include <Utils/Math/minjerk_one_dim.hpp>

class BodyFootLearningCtrl : public SwingPlanningCtrl {
   public:
    BodyFootLearningCtrl(RobotSystem* robot, int swing_foot,
                         TVRPlanner* planner);
    virtual ~BodyFootLearningCtrl();
    virtual void oneStep(void* _cmd);
    virtual void firstVisit();
    virtual void lastVisit() { sp_->des_jpos_prev = des_jpos_; }
    virtual bool endOfPhase();
    virtual void ctrlInitialization(const YAML::Node& node);

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
    void setTerminateObsLowerBound(Eigen::VectorXd lb) {
        terminate_obs_lower_bound_ = lb;
    }
    void setTerminateObsUpperBound(Eigen::VectorXd ub) {
        terminate_obs_upper_bound_ = ub;
    }
    void setActionScale(Eigen::VectorXd as) { action_scale_ = as; }
    void setPolicy(NeuralNetModel* model) { nn_policy_ = model; }
    void setValueFn(NeuralNetModel* model) { nn_valfn_ = model; }

   protected:
    double waiting_time_limit_;
    double ini_base_height_;
    int swing_leg_jidx_;
    double push_down_height_;  // push foot below the ground at landing

    Eigen::Vector3d default_target_loc_;
    Eigen::Vector3d initial_target_loc_;

    // rl parameters
    Eigen::VectorXd action_lower_bound_;
    Eigen::VectorXd action_upper_bound_;
    Eigen::MatrixXd action_lower_bound_mat_;
    Eigen::MatrixXd action_upper_bound_mat_;
    Eigen::VectorXd terminate_obs_lower_bound_;
    Eigen::VectorXd terminate_obs_upper_bound_;
    double quad_input_penalty_;
    double alive_reward_;
    double deviation_penalty_;
    double reward_scale_;
    Eigen::VectorXd action_scale_;

    NeuralNetModel* nn_policy_;
    NeuralNetModel* nn_valfn_;

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
