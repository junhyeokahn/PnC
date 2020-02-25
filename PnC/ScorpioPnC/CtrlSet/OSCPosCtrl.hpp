#pragma once

#include <PnC/Controller.hpp>

class RobotSystem;

class OSCPosCtrl : public Controller {
   public:
    OSCPosCtrl(RobotSystem* _robot);
    virtual ~OSCPosCtrl();

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

   protected:
    double end_time_;
    int ctrl_count_;
    int task_dim_;
    std::vector<int> active_joint_idx_;
    Eigen::VectorXd target_pos_;
    Eigen::VectorXd ini_pos_;
    Eigen::VectorXd ini_vel_;
    Eigen::VectorXd ini_pos_q;
    Eigen::VectorXd ini_vel_q;
    Eigen::VectorXd q_kp_;
    Eigen::VectorXd q_kd_;
    Eigen::VectorXd end_effector_kp_;
    Eigen::VectorXd end_effector_kd_;
    Eigen::Quaternion<double> ini_ori_;
    Eigen::Quaternion<double> target_ori_;
    Eigen::VectorXd ori_err_;
    
    Eigen::VectorXd des_pos_data_;
    Eigen::VectorXd act_pos_data_;
    Eigen::VectorXd des_vel_data_;
    Eigen::VectorXd act_vel_data_;

    Eigen::VectorXd ori_err_data_;
};
