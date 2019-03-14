#pragma once

#include <PnC/Controller.hpp>

class RobotSystem;

class JPosSwingCtrl : public Controller {
    public:
    JPosSwingCtrl(RobotSystem* _robot);
    virtual ~JPosSwingCtrl();

    void oneStep(void* _cmd);
    void firstVisit();
    void lastVisit();
    bool endOfPhase();
    void ctrlInitialization(const YAML::Node& node);

    
    void setEndTime(double time) { end_time_ = time;}
    void setInitialPosition(const Eigen::VectorXd& _set_ini_jpos) {
    set_ini_jpos_ = _set_ini_jpos;}
    void setAmplitude(const Eigen::VectorXd& amp){
    amp_ = amp;} 
    void setFrequency(const Eigen::VectorXd& freq){
    freq_ = freq;} 
    void setPhase(const Eigen::VectorXd& phase){
    phase_ = phase;} 
    

    private:
    double end_time_;
    int ctrl_count_;
   

    Eigen::VectorXd set_ini_jpos_;
    
    //Eigen::VectorXd ini_pos_;
    //Eigen::VectorXd ini_vel_;

    Eigen::VectorXd amp_;
    Eigen::VectorXd freq_;
    Eigen::VectorXd phase_;
    Eigen::VectorXd kp_;
    Eigen::VectorXd kd_; 

};

