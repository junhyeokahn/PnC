#pragma once

#include <array>

#include <Utils/Math/BSplineBasic.h>
#include <PnC/Controller.hpp>
#include <PnC/PlannerSet/CentroidPlanner/CentroidPlanner.hpp>

class DracoStateProvider;
class RobotSystem;
class WBLC;
class WBLC_ExtraData;
class KinWBC;
class ContactSpec;

class TransitionCtrl : public Controller {
   public:
    TransitionCtrl(RobotSystem*, Planner*, int moving_foot, bool b_increase);
    virtual ~TransitionCtrl();

    virtual void oneStep(void* _cmd);
    virtual void firstVisit();
    virtual void lastVisit();
    virtual bool endOfPhase();
    virtual void ctrlInitialization(const YAML::Node& node);

    void SetTRNSDuration(double time) { trns_dur_ = time; }
    void SetCoMHeight(double h) { com_height_ = h; }
    void SetDSPDuration(double time) { dsp_dur_ = time; }
    void SetSSPDuration(double time) { ssp_dur_ = time; }
    void SetIniDSPDuration(double time) { ini_dsp_dur_ = time; }

   protected:
    double max_rf_z_;
    double min_rf_z_;

    Eigen::VectorXd Kp_, Kd_;
    Eigen::VectorXd des_jpos_;
    Eigen::VectorXd des_jvel_;
    Eigen::VectorXd des_jacc_;

    Eigen::VectorXd jpos_ini_;

    double trns_dur_;
    double com_height_;
    int dim_contact_;

    double ini_dsp_dur_;
    double dsp_dur_;
    double ssp_dur_;
    bool b_increase_;
    int moving_foot_;
    int stance_foot_;
    int moving_cop_;
    int stance_cop_;

    Task* com_task_;
    Task* total_joint_task_;
    Task* torso_ori_task_;

    ContactSpec* rfoot_front_contact_;
    ContactSpec* rfoot_back_contact_;
    ContactSpec* lfoot_front_contact_;
    ContactSpec* lfoot_back_contact_;

    //ContactSpec* rfoot_contact_;
    //ContactSpec* lfoot_contact_;

    KinWBC* kin_wbc_;
    WBLC* wblc_;
    WBLC_ExtraData* wblc_data_;

    void _task_setup();
    void _contact_setup();
    void _compute_torque_wblc(Eigen::VectorXd& gamma);

    double ctrl_start_time_;
    DracoStateProvider* sp_;

    Eigen::VectorXd ini_com_pos_;

    Planner* planner_;
};
