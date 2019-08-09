#pragma once

#include <array>

#include <Utils/Math/BSplineBasic.h>
#include <PnC/Controller.hpp>
#include <PnC/PlannerSet/CentroidPlanner/CentroidPlanner.hpp>

class ValkyrieStateProvider;
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
    void SetFootStepLength(double l) { footstep_length_ = l; }
    void SetFootStepWidth(double w) { footstep_width_ = w; }
    void SetCoMHeight(double h) { com_height_ = h; }
    void SetRePlanningFlag(bool b) { b_replan_ = b; }

   protected:
    Eigen::VectorXd Kp_, Kd_;
    Eigen::VectorXd des_jpos_;
    Eigen::VectorXd des_jvel_;
    Eigen::VectorXd des_jacc_;

    Eigen::VectorXd jpos_ini_;

    double dsp_dur_;
    double ssp_dur_;
    double footstep_length_;
    double footstep_width_;
    double com_height_;
    bool b_replan_;
    bool b_do_plan_;

    int dim_contact_;

    int moving_foot_;
    int stance_foot_;
    int moving_cop_;
    int stance_cop_;

    Task* centroid_task_;
    Task* com_task_;
    Task* pelvis_ori_task_;
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

    void PlannerUpdate_();
    void PlannerInitialization_();
    void _task_setup();
    void _contact_setup();
    void _compute_torque_wblc(Eigen::VectorXd& gamma);

    double ctrl_start_time_;
    ValkyrieStateProvider* sp_;

    BS_Basic<3, 3, 1, 2, 2> foot_traj_;
    void SetBSpline_();

    Planner* planner_;
    Eigen::MatrixXd com_traj_;
    Eigen::MatrixXd lmom_traj_;
    Eigen::MatrixXd amom_traj_;
    std::array<Eigen::MatrixXd, CentroidModel::numEEf> cop_local_traj_;
    std::array<Eigen::MatrixXd, CentroidModel::numEEf> frc_world_traj_;
    std::array<Eigen::MatrixXd, CentroidModel::numEEf> trq_local_traj_;
};
