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

class SingleSupportCtrl : public Controller {
   public:
    SingleSupportCtrl(RobotSystem*, Planner* planner, int moving_foot);
    virtual ~SingleSupportCtrl();

    virtual void oneStep(void* _cmd);
    virtual void firstVisit();
    virtual void lastVisit();
    virtual bool endOfPhase();
    virtual void ctrlInitialization(const YAML::Node& node);

    void SetDSPDuration(double time) { dsp_dur_ = time; }
    void SetSSPDuration(double time) { ssp_dur_ = time; }
    void SetCoMHeight(double h) { com_height_ = h; }
    void SetSwingHeight(double h) { swing_height_ = h; }

   protected:
    Eigen::VectorXd Kp_, Kd_;
    Eigen::VectorXd des_jpos_;
    Eigen::VectorXd des_jvel_;
    Eigen::VectorXd des_jacc_;

    Eigen::VectorXd jpos_ini_;

    double swing_height_;
    double dsp_dur_;
    double ssp_dur_;
    double com_height_;

    int dim_contact_;

    int moving_foot_;
    int stance_foot_;
    int moving_cop_;
    int stance_cop_;

    Eigen::Quaternion<double> ini_quat_torso_;
    Eigen::Quaternion<double> ini_quat_foot_;
    Eigen::Quaternion<double> des_quat_;

    Task* com_task_;
    Task* torso_ori_task_;
    Task* foot_pos_task_;
    Task* foot_ori_task_;
    Task* total_joint_task_;

    ContactSpec* rfoot_contact_;
    ContactSpec* lfoot_contact_;
    std::vector<ContactSpec*> kin_wbc_contact_list_;

    KinWBC* kin_wbc_;
    WBLC* wblc_;
    WBLC_ExtraData* wblc_data_;

    void _task_setup();
    void _contact_setup();
    void _compute_torque_wblc(Eigen::VectorXd& gamma);

    double ctrl_start_time_;
    DracoStateProvider* sp_;

    BS_Basic<3, 3, 1, 2, 2> foot_traj_;
    void SetBSpline_();

    Planner* planner_;
};
