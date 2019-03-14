#pragma once

#include <PnC/Controller.hpp>

class RobotSystem;
class AtlasStateProvider;
class ContactSpec;
class WBLC;
class KinWBC;
class WBLC_ExtraData;

class SingleContactTransCtrl : public Controller {
   public:
    SingleContactTransCtrl(RobotSystem* robot, int moving_foot,
                           bool b_increase);
    virtual ~SingleContactTransCtrl();

    virtual void oneStep(void* _cmd);
    virtual void firstVisit();
    virtual void lastVisit();
    virtual bool endOfPhase();
    virtual void ctrlInitialization(const YAML::Node& node);

    void setTransitionTime(double time) { end_time_ = time; }
    void setStanceHeight(double height) {
        target_body_height_ = height;
        b_set_height_target_ = true;
    }

   protected:
    bool b_set_height_target_;
    double target_body_height_;
    double ini_base_height_;
    Eigen::VectorXd ini_body_pos_;
    int dim_contact_;

    double end_time_;
    int moving_foot_;
    bool b_increase_;  // Increasing or decreasing reaction force
    double max_rf_z_;
    double min_rf_z_;

    std::vector<int> selected_jidx_;
    Task* body_pos_task_;  // pelvis
    Task* body_ori_task_;
    Task* torso_ori_task_;
    Task* total_joint_task_;

    KinWBC* kin_wbc_;
    ContactSpec* rfoot_contact_;
    ContactSpec* lfoot_contact_;
    WBLC* wblc_;
    WBLC_ExtraData* wblc_data_;

    Eigen::VectorXd des_jpos_;
    Eigen::VectorXd des_jvel_;
    Eigen::VectorXd des_jacc_;

    Eigen::VectorXd Kp_;
    Eigen::VectorXd Kd_;

    void _task_setup();
    void _contact_setup();
    void _compute_torque_wblc(Eigen::VectorXd& gamma);

    AtlasStateProvider* sp_;
    double ctrl_start_time_;
};
