#pragma once

#include <PnC/Test.hpp>

class AtlasStateProvider;
class FootStepPlanner;

namespace RLWkPhase {
constexpr int double_contact_1 = 0;
constexpr int right_swing_start_trans = 1;
constexpr int right_swing = 2;
constexpr int right_swing_end_trans = 3;
constexpr int double_contact_2 = 4;
constexpr int left_swing_start_trans = 5;
constexpr int left_swing = 6;
constexpr int left_swing_end_trans = 7;
constexpr int NUM_WALKING_PHASE = 8;
};  // namespace RLWkPhase

class RLWalkingTest : public Test {
   public:
    RLWalkingTest(RobotSystem*);
    RLWalkingTest(RobotSystem*, int mpi_idx, int env_idx);
    virtual ~RLWalkingTest();
    virtual void TestInitialization();

   protected:
    int num_step_;
    AtlasStateProvider* sp_;
    virtual int _NextPhase(const int& phase);
    virtual void AdditionalUpdate_();
    void _SettingParameter();

    int mpi_idx_;
    int env_idx_;
    bool b_learning_;

    FootStepPlanner* reversal_planner_;

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

    // Locomotion Behavior
    double walking_start_time_;
    Eigen::Vector2d walking_velocity_;
    double turning_rate_;
    Eigen::Quaternion<double> delta_quat_;
};
