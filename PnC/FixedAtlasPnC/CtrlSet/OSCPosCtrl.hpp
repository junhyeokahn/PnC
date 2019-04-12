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

   protected:
    double end_time_;
    int ctrl_count_;
    Eigen::VectorXd target_pos_;
    Eigen::VectorXd ini_pos_;
    Eigen::VectorXd ini_vel_;
    Eigen::VectorXd ini_pos_q;
    Eigen::VectorXd ini_vel_q;
    Eigen::VectorXd q_kp_;
    Eigen::VectorXd q_kd_;
    Eigen::VectorXd rf_kp_;
    Eigen::VectorXd rf_kd_;

};
