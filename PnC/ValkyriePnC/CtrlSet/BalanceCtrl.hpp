#pragma once

#include <PnC/Controller.hpp>
#include <Utils/Math/BSplineBasic.h>

class ValkyrieStateProvider;
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
    
    void setDuration(double time){end_time_ = time;}
    void setComDeviation(const Eigen::VectorXd& dev){com_pos_dev_ = dev;}

   protected:
    Task* com_task_;
    Task* torso_ori_task_;
    Task* total_joint_task_;

    ContactSpec* rfoot_contact_;
    ContactSpec* lfoot_contact_;

    KinWBC* kin_wbc_;
    WBLC* wblc_;
    WBLC_ExtraData* wblc_data_;

    void _task_setup();
    void _contact_setup();
    void _compute_torque_wblc(Eigen::VectorXd& gamma);
    void _SetBspline(const Eigen::VectorXd st_pos,
                     const Eigen::VectorXd des_pos); 
    void _GetBsplineTrajectory();

    double ctrl_start_time_;
    double end_time_;
    ValkyrieStateProvider* sp_;

    Eigen::VectorXd jpos_ini_;
    Eigen::VectorXd Kp_, Kd_;
    Eigen::VectorXd des_jpos_;
    Eigen::VectorXd des_jvel_;
    Eigen::VectorXd des_jacc_;
    int dim_contact_;
    

    Eigen::VectorXd ini_com_pos_;
    Eigen::VectorXd des_com_pos_;
    Eigen::VectorXd des_com_vel_;
    Eigen::VectorXd des_com_acc_;
    
    Eigen::VectorXd com_pos_dev_;

    BS_Basic<3,3,1,2,2> com_traj_;

    Eigen::Quaternion<double> ini_torso_quat;
};
