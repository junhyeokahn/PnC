#pragma once

#include <PnC/Controller.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>

class DracoStateProvider;
class FootStepPlanner;
class ContactSpec;
class WBLC;
class WBLC_ExtraData;

class SwingPlanningCtrl : public Controller {
   public:
    SwingPlanningCtrl(RobotSystem* robot, std::string swing_foot,
                      FootStepPlanner* planner)
        : Controller(robot),
          swing_foot_(swing_foot),
          b_replanning_(false),
          planner_(planner),
          planning_frequency_(0.),
          replan_moment_(0.),
          ctrl_start_time_(0.),
          half_swing_time_(0.15),
          b_set_height_target_(false),
          b_contact_switch_check_(false) {
        curr_foot_pos_des_.setZero();
        curr_foot_vel_des_.setZero();
        curr_foot_acc_des_.setZero();
        curr_foot_so3_des_.setZero();
        curr_foot_quat_des_ = Eigen::Quaternion<double>(1., 0., 0., 0.);
        sp_ = DracoStateProvider::getStateProvider(robot_);
    }

    virtual ~SwingPlanningCtrl() {}

    void setReplanning(bool replan) { b_replanning_ = replan; }
    void setSwingTime(double swing_time) {
        end_time_ = swing_time;
        half_swing_time_ = end_time_ / 2.;
    }
    void setDoubleStanceRatio(double ratio) { double_stance_ratio_ = ratio; }
    void setTransitionPhaseRatio(double ratio) {
        transition_phase_ratio_ = ratio;
    }

    void notifyTransitionTime(double time) { transition_time_ = time; }
    void notifyStanceTime(double time) { stance_time_ = time; }

    void setStanceHeight(double height) {
        des_body_height_ = height;
        b_set_height_target_ = true;
    }
    void setContactSwitchCheck(bool switch_check) {
        b_contact_switch_check_ = switch_check;
    }

    Eigen::Vector3d curr_foot_pos_des_;
    Eigen::Vector3d curr_foot_vel_des_;
    Eigen::Vector3d curr_foot_acc_des_;
    Eigen::Quaternion<double> curr_foot_quat_des_;
    Eigen::Vector3d curr_foot_so3_des_;

   protected:
    bool b_contact_switch_check_;
    bool b_set_height_target_;
    double des_body_height_;

    double double_stance_ratio_;
    double transition_phase_ratio_;
    double replan_moment_;

    std::string swing_foot_;
    double swing_height_;
    Eigen::Vector3d default_target_loc_;

    double planning_frequency_;
    bool b_replanning_;
    bool b_replaned_;

    ContactSpec* single_contact_;
    WBLC* wblc_;
    WBLC_ExtraData* wblc_data_;
    FootStepPlanner* planner_;

    DracoStateProvider* sp_;

    // Timing parameters
    double end_time_;
    double half_swing_time_;
    double transition_time_;
    double stance_time_;
    double ctrl_start_time_;
};
