#pragma once

#include <array>

#include <Utils/Math/BSplineBasic.h>
#include <PnC/Controller.hpp>

class ValkyrieStateProvider;
class RobotSystem;
class WBLC;
class WBLC_ExtraData;
class KinWBC;
class ContactSpec;

class TransitionCtrl : public Controller {
   public:
    TransitionCtrl(RobotSystem*, int moving_foot, bool b_increase);
    virtual ~TransitionCtrl();

    virtual void oneStep(void* _cmd);
    virtual void firstVisit();
    virtual void lastVisit();
    virtual bool endOfPhase();
    virtual void ctrlInitialization(const YAML::Node& node);

    void SetTRNSDuration(double time) { trns_dur_ = time; }
    void SetCoMHeight(double h) { com_height_ = h; }

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

    bool b_increase_;
    int moving_foot_;
    int stance_foot_;
    int moving_cop_;
    int stance_cop_;

    Task* centroid_task_;
    Task* total_joint_task_;

    ContactSpec* rfoot_contact_;
    ContactSpec* lfoot_contact_;

    KinWBC* kin_wbc_;
    WBLC* wblc_;
    WBLC_ExtraData* wblc_data_;

    void _task_setup();
    void _contact_setup();
    void _compute_torque_wblc(Eigen::VectorXd& gamma);

    double ctrl_start_time_;
    ValkyrieStateProvider* sp_;

    BS_Basic<3, 3, 1, 2, 2> com_traj_;
    void SetBSpline_();
};
