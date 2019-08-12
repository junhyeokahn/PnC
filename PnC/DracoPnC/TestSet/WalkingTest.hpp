#pragma once

#include <PnC/Test.hpp>

class DracoStateProvider;
class TVRPlanner;

namespace WkPhase {
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
};  // namespace WkPhase

class WalkingTest : public Test {
   public:
    WalkingTest(RobotSystem*);
    virtual ~WalkingTest();
    virtual void TestInitialization();

   protected:
    int num_step_;
    DracoStateProvider* sp_;
    virtual int _NextPhase(const int& phase);
    void _SettingParameter();

    TVRPlanner* reversal_planner_;

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
};
