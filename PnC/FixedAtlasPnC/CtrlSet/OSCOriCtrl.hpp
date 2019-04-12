#pragma once

#include <PnC/Controller.hpp>

class RobotSystem;

class OSCOriCtrl : public Controller {
   public:
    OSCOriCtrl(RobotSystem* _robot);
    virtual ~OSCOriCtrl();

    virtual void oneStep(void* _cmd);
    virtual void firstVisit();
    virtual void lastVisit();
    virtual bool endOfPhase();
    virtual void ctrlInitialization(const YAML::Node& node);

    void setEndTime(double time) { end_time_ = time; }
    void setTargetPosition(const Eigen::VectorXd& pos) 
    { target_pos_ = pos;}

   protected:
    double end_time_;
    int ctrl_count_;
    Eigen::VectorXd target_pos_;
    Eigen::Quaternion<double> ini_pos_quat_;
    Eigen::Quaternion<double> fin_pos_quat_;
    Eigen::Quaternion<double> quat_ori_error;
    Eigen::Vector3d so3_ori_error;
    Eigen::VectorXd head_kp_;
    Eigen::VectorXd head_kd_;
    Eigen::VectorXd q_kp_;
    Eigen::VectorXd q_kd_;
    Eigen::VectorXd ini_pos_q;
    Eigen::VectorXd ini_vel_q;
};
