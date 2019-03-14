#pragma once

#include <PnC/Test.hpp>

class DracoStateProvider;
class FootStepPlanner;

namespace RlWkPhase {
constexpr int initiation = 0;
constexpr int lift_up = 1;
constexpr int double_contact_1 = 2;
constexpr int right_swing_start_trans = 3;
constexpr int right_swing = 4;
constexpr int right_swing_end_trans = 5;
constexpr int double_contact_2 = 6;
constexpr int left_swing_start_trans = 7;
constexpr int left_swing = 8;
constexpr int left_swing_end_trans = 9;
constexpr int NUM_WALKING_PHASE = 10;
};  // namespace RlWkPhase

class RLWalkingTest : public Test {
   public:
    RLWalkingTest(RobotSystem*);
    RLWalkingTest(RobotSystem*, int mpi_idx, int env_idx);
    virtual ~RLWalkingTest();
    virtual void TestInitialization();

   protected:
    int num_step_;
    int mpi_idx_;
    int env_idx_;
    bool b_learning_;

    DracoStateProvider* sp_;
    virtual int _NextPhase(const int& phase);
    virtual void AdditionalUpdate_();
    void _SettingParameter();

    FootStepPlanner* reversal_planner_;

    Controller* jpos_ctrl_;
    Controller* body_up_ctrl_;
    Controller* body_fix_ctrl_;
    // Right
    Controller* right_swing_start_trans_ctrl_;
    Controller* right_swing_ctrl_;
    Controller* right_swing_end_trans_ctrl_;
    // Left
    Controller* left_swing_start_trans_ctrl_;
    Controller* left_swing_ctrl_;
    Controller* left_swing_end_trans_ctrl_;

    YAML::Node cfg_;

    // walking
    double walking_start_time_;
    Eigen::Vector2d walking_velocity_;
};
