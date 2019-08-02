#pragma once

#include <PnC/Controller.hpp>

class AtlasStateProvider;
class RobotSystem;
class WBLC;
class WBLC_ExtraData;
class KinWBC;
class ContactSpec;
class Planner;

class DoubleSupportCtrl : public Controller {
   public:
    DoubleSupportCtrl(RobotSystem*, Planner*);
    virtual ~DoubleSupportCtrl();

    virtual void oneStep(void* _cmd);
    virtual void firstVisit();
    virtual void lastVisit();
    virtual bool endOfPhase();
    virtual void ctrlInitialization(const YAML::Node& node);

    void SetDSPDuration(double time) { dsp_dur_ = time; }
    void SetIniDSPDuration(double time) { ini_dsp_dur_ = time; }
    void SetSSPDuration(double time) { ssp_dur_ = time; }
    void SetFootStepLength(Eigen::VectorXd fsl) { footstep_length_ = fsl; }
    void SetCoMHeight(double h) { com_height_ = h; }

   protected:
    Eigen::VectorXd Kp_, Kd_;
    Eigen::VectorXd des_jpos_;
    Eigen::VectorXd des_jvel_;
    Eigen::VectorXd des_jacc_;

    Eigen::VectorXd jpos_ini_;

    Eigen::VectorXd footstep_length_;
    double ini_dsp_dur_;
    double dsp_dur_;
    double ssp_dur_;
    double com_height_;
    int dim_contact_;

    std::vector<int> selected_jidx_;

    Task* total_joint_task_;

    ContactSpec* rfoot_contact_;
    ContactSpec* lfoot_contact_;

    KinWBC* kin_wbc_;
    WBLC* wblc_;
    WBLC_ExtraData* wblc_data_;

    // Task specification
    Eigen::VectorXd ini_body_pos_;

    Eigen::Quaternion<double> ini_body_quat_;
    Eigen::Quaternion<double> body_delta_quat_;
    Eigen::Vector3d body_delta_so3_;

    Eigen::Quaternion<double> ini_torso_quat_;
    Eigen::Quaternion<double> torso_delta_quat_;
    Eigen::Vector3d torso_delta_so3_;

    void PlannerInitialization_();
    void _task_setup();
    void _contact_setup();
    void _compute_torque_wblc(Eigen::VectorXd& gamma);

    double ctrl_start_time_;
    AtlasStateProvider* sp_;

    Planner* planner_;
};
