#pragma once

#include <PnC/Controller.hpp>

class DracoStateProvider;
class RobotSystem;
class WBLC;
class WBLC_ExtraData;
class KinWBC;
class ContactSpec;

class CoMCtrl : public Controller {
   public:
    CoMCtrl(RobotSystem*);
    virtual ~CoMCtrl();

    virtual void oneStep(void* _cmd);
    virtual void firstVisit();
    virtual void lastVisit();
    virtual bool endOfPhase();
    virtual void ctrlInitialization(const YAML::Node& node);

    void setStanceTime(double time) { end_time_ = time; }
    void setCoMHeight(double height) {
        target_com_height_ = height;
    }
    void SetStabilizationDuration(double time){
        stab_dur_ = time;
    }

   protected:
    Eigen::VectorXd Kp_, Kd_;
    Eigen::VectorXd des_jpos_;
    Eigen::VectorXd des_jvel_;
    Eigen::VectorXd des_jacc_;

    Eigen::VectorXd jpos_ini_;
    double end_time_;
    int dim_contact_;

    Task* com_task_;
    Task* total_joint_task_;

    ContactSpec* rfoot_front_contact_;
    ContactSpec* rfoot_back_contact_;
    ContactSpec* lfoot_front_contact_;
    ContactSpec* lfoot_back_contact_;

    KinWBC* kin_wbc_;
    WBLC* wblc_;
    WBLC_ExtraData* wblc_data_;

    double target_com_height_;
    double stab_dur_;
    Eigen::VectorXd ini_com_pos_;
    Eigen::VectorXd ini_com_vel_;
    Eigen::VectorXd goal_com_pos_;

    void task_setup();
    void contact_setup();
    void _compute_torque_wblc(Eigen::VectorXd& gamma);

    double ctrl_start_time_;
    DracoStateProvider* sp_;

    double des_com_offset_x_;
};
