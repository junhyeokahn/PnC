#pragma once

#include <PnC/Controller.hpp>

class RobotSystem;

class ScorpioStateProvider;
class OSC;
class Task;

class OSCCtrl : public Controller {
    public:
        OSCCtrl(RobotSystem* _robot);
        virtual ~OSCCtrl();

        virtual void oneStep(void* _cmd);
        virtual void firstVisit();
        virtual void lastVisit();
        virtual bool endOfPhase();
        virtual void ctrlInitialization(const YAML::Node& node);

        void setEndTime(double time) { end_time_ = time; }
        void setTargetPosition(const Eigen::VectorXd& pos)
        { target_pos_ = pos;}
        void setTargetOrientation(const Eigen::VectorXd& ori) 
        { target_ori_.w() = ori[0];
            target_ori_.x() = ori[1];
            target_ori_.y() = ori[2];
            target_ori_.z() = ori[3];}
        void setRelativeTargetPosition(const Eigen::VectorXd& pos)
        { relative_target_pos_ = pos;}
        void setRelativeTargetOrientation(const Eigen::VectorXd& ori) 
        { relative_target_ori_.w() = ori[0];
          relative_target_ori_.x() = ori[1];
          relative_target_ori_.y() = ori[2];
          relative_target_ori_.z() = ori[3];}

    protected:
        double end_time_;
        Eigen::VectorXd relative_target_pos_;
        Eigen::Quaternion<double> relative_target_ori_;
        Eigen::VectorXd target_pos_;
        Eigen::VectorXd ini_pos_;
        Eigen::VectorXd ini_pos_q_;
        Eigen::VectorXd q_kp_;
        Eigen::VectorXd q_kd_;
        Eigen::VectorXd lin_kp_;
        Eigen::VectorXd lin_kd_;
        Eigen::VectorXd ori_kp_;
        Eigen::VectorXd ori_kd_;
        Eigen::Quaternion<double> ini_ori_;
        Eigen::Quaternion<double> target_ori_;

        Task* ee_pos_task_;
        Task* ee_ori_task_;
        Task* joint_task_;

        OSC* osc_;

        void _build_constraint_matrix();
        void _build_active_joint_idx();
        void _task_setup();
        void _compute_torque(Eigen::VectorXd & gamma);

        std::vector<int> active_joint_idx_;
        std::vector<bool> active_joint_;
        Eigen::MatrixXd Jc_;
        ScorpioStateProvider* sp_;

        double ctrl_start_time_;
};
