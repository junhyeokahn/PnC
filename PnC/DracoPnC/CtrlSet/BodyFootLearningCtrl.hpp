#pragma once

#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <Utils/Math/minjerk_one_dim.hpp>
#include <Utils/Math/BSplineBasic.h>
#include <ReinforcementLearning/RLInterface/NeuralNetModel.hpp>

class KinWBC;

class BodyFootLearningCtrl:public SwingPlanningCtrl{
   public:
        BodyFootLearningCtrl(RobotSystem* robot,
                std::string swing_foot_, FootStepPlanner* planner);
        virtual ~BodyFootLearningCtrl();
        virtual void oneStep(void* _cmd);
        virtual void firstVisit();
        virtual void lastVisit(){
            sp_->des_jpos_prev = des_jpos_;
            sp_->contact_time = sp_->curr_time;
            sp_->b_observe_keyframe_vel[0] = true;
            sp_->b_observe_keyframe_vel[1] = true;
            //std::cout << "[BodyFootPlanning] End "<< std::endl;
        }
        virtual bool endOfPhase();

        virtual void ctrlInitialization(const YAML::Node& node);

        void setActionLowerBound(Eigen::VectorXd lb) {action_lower_bound_ = lb;}
        void setActionUpperBound(Eigen::VectorXd ub) {action_upper_bound_ = ub;}
        void setTerminateObsLowerBound(Eigen::VectorXd lb) {terminate_obs_lower_bound_ = lb;}
        void setTerminateObsUpperBound(Eigen::VectorXd ub) {terminate_obs_upper_bound_ = ub;}
        void setActionScale(Eigen::VectorXd as) {action_scale_ = as;}
        void setPolicy(NeuralNetModel* model) { nn_policy_ = model; }
        void setValueFn(NeuralNetModel* model) { nn_valfn_ = model; }
    protected:
        double ini_base_height_;
        int swing_leg_jidx_;
        double push_down_height_; // push foot below the ground at landing
        Eigen::Vector3d initial_target_loc_;

        // rl parameters
        Eigen::VectorXd action_lower_bound_;
        Eigen::VectorXd action_upper_bound_;
        Eigen::VectorXd terminate_obs_lower_bound_;
        Eigen::VectorXd terminate_obs_upper_bound_;
        double quad_input_penalty_;
        double alive_reward_;
        double deviation_penalty_;
        double reward_scale_;
        Eigen::VectorXd action_scale_;

        int dim_contact_;
        // [right_front, right_back, left_front, left_back]
        std::vector<int> fz_idx_in_cost_;
        ContactSpec* rfoot_contact_;
        ContactSpec* lfoot_contact_;

        void _CheckPlanning();
        void _Replanning(Eigen::Vector3d & target_loc);
        void _contact_setup();
        void _task_setup();
        void _compute_torque_wblc(Eigen::VectorXd & gamma);
        void _SetMinJerkOffset(const Eigen::Vector3d & offset);
        void _SetBspline(
            const Eigen::Vector3d & st_pos,
            const Eigen::Vector3d & des_pos);

        void _GetSinusoidalSwingTrajectory();
        void _GetBsplineSwingTrajectory();
        std::vector<ContactSpec*> kin_wbc_contact_list_;

        std::vector<int> selected_jidx_;
        Task* selected_joint_task_;
        Task* base_task_;
        //Task* foot_pitch_task_;
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

        NeuralNetModel* nn_policy_;
        NeuralNetModel* nn_valfn_;
};
