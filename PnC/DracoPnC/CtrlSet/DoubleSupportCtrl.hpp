#pragma once

#include <array>

#include <PnC/Controller.hpp>
#include <PnC/PlannerSet/CentroidPlanner/CentroidPlanner.hpp>

class DracoStateProvider;
class RobotSystem;
class WBLC;
class WBLC_ExtraData;
class KinWBC;
class ContactSpec;
class FootstepSequenceGenerator;

class DoubleSupportCtrl : public Controller {
   public:
    DoubleSupportCtrl(RobotSystem*, Planner*, FootstepSequenceGenerator*);
    virtual ~DoubleSupportCtrl();

    virtual void oneStep(void* _cmd);
    virtual void firstVisit();
    virtual void lastVisit();
    virtual bool endOfPhase();
    virtual void ctrlInitialization(const YAML::Node& node);

    void SetDSPDuration(double time) { dsp_dur_ = time; }
    void SetIniDSPDuration(double time) { ini_dsp_dur_ = time; }
    void SetFinDSPDuration(double time) { fin_dsp_dur_ = time; }
    void SetSSPDuration(double time) { ssp_dur_ = time; }
    void SetCoMHeight(double h) { com_height_ = h; }

   protected:
    Eigen::VectorXd Kp_, Kd_;
    Eigen::VectorXd des_jpos_;
    Eigen::VectorXd des_jvel_;
    Eigen::VectorXd des_jacc_;

    Eigen::VectorXd jpos_ini_;
    Eigen::Vector3d ini_com_pos_;
    Eigen::Vector3d ini_com_vel_;
    Eigen::Vector3d goal_com_pos_;

    double ini_dsp_dur_;
    double fin_dsp_dur_;
    double dsp_dur_;
    double ssp_dur_;
    double com_height_;
    int dim_contact_;

    bool b_do_plan_;

    Task* com_task_;
    Task* torso_RxRy_task_;
    Task* torso_ori_task_;
    Task* total_joint_task_;

    ContactSpec* rfoot_contact_;
    ContactSpec* lfoot_contact_;


    // TEST
    //ContactSpec* rfoot_front_contact_;
    //ContactSpec* rfoot_back_contact_;
    //ContactSpec* lfoot_front_contact_;
    //ContactSpec* lfoot_back_contact_;
    // TEST

    KinWBC* kin_wbc_;
    WBLC* wblc_;
    WBLC_ExtraData* wblc_data_;

    void PlannerUpdate_();
    void PlannerInitialization_();
    void _balancing_task_setup();
    void _walking_task_setup();
    void _contact_setup();
    void _compute_torque_wblc(Eigen::VectorXd& gamma);

    double ctrl_start_time_;
    DracoStateProvider* sp_;

    FootstepSequenceGenerator* foot_sequence_gen_;
    Planner* planner_;
    Eigen::MatrixXd com_traj_;
    Eigen::MatrixXd lmom_traj_;
    Eigen::MatrixXd amom_traj_;
    std::array<Eigen::MatrixXd, CentroidModel::numEEf> cop_local_traj_;
    std::array<Eigen::MatrixXd, CentroidModel::numEEf> frc_world_traj_;
    std::array<Eigen::MatrixXd, CentroidModel::numEEf> trq_local_traj_;

    void AddContactSequence_(
        CentroidModel::EEfID eef_id, double ini, double fin,
        Eigen::Isometry3d f_iso,
        std::array<std::vector<Eigen::VectorXd>, CentroidModel::numEEf>& c_seq);
};
