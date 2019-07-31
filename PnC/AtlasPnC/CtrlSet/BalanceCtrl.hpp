#pragma once

#include <PnC/Controller.hpp>

class AtlasStateProvider;
class RobotSystem;
class WBLC;
class WBLC_ExtraData;
class KinWBC;
class ContactSpec;

class BalanceCtrl : public Controller {
   public:
    BalanceCtrl(RobotSystem*);
    virtual ~BalanceCtrl();

    virtual void oneStep(void* _cmd);
    virtual void firstVisit();
    virtual void lastVisit();
    virtual bool endOfPhase();
    virtual void ctrlInitialization(const YAML::Node& node);

   protected:
    // Task* com_task_;
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
    AtlasStateProvider* sp_;

    Eigen::VectorXd jpos_ini_;
    Eigen::VectorXd Kp_, Kd_;
    Eigen::VectorXd des_jpos_;
    Eigen::VectorXd des_jvel_;
    Eigen::VectorXd des_jacc_;
    int dim_contact_;
};
