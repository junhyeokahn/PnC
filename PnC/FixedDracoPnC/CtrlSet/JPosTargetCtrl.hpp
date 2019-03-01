#pragma once

#include <PnC/Controller.hpp>

class RobotSystem;

class JPosTargetCtrl : public Controller {
   public:
    JPosTargetCtrl(RobotSystem* _robot);
    virtual ~JPosTargetCtrl();

    virtual void oneStep(void* _cmd);
    virtual void firstVisit();
    virtual void lastVisit();
    virtual bool endOfPhase();
    virtual void ctrlInitialization(const YAML::Node& node);

    void setMovingTime(double time) { move_time_ = time; }
    void setEndTime(double time) { end_time_ = time; }
    void setTargetPosition(const Eigen::VectorXd& jpos) { target_pos_ = jpos; };

   protected:
    double move_time_;
    double end_time_;
    double ctrl_start_time_;
    int ctrl_count_;
    double ctrl_time_;
    Eigen::VectorXd target_pos_;
    Eigen::VectorXd ini_pos_;
    Eigen::VectorXd ini_vel_;
    Eigen::VectorXd kp_;
    Eigen::VectorXd kd_;
};
