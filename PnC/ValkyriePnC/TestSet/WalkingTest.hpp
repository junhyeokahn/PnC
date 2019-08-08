#pragma once

#include <PnC/Test.hpp>

class ValkyrieStateProvider;
class Planner;
class CentroidPlannerParameter;

namespace WKPhase {
constexpr int double_contact_1 = 0;
constexpr int right_swing_start_trans = 1;
constexpr int right_swing = 2;
constexpr int right_swing_end_trans = 3;
constexpr int double_contact_2 = 4;
constexpr int left_swing_start_trans = 5;
constexpr int left_swing = 6;
constexpr int left_swing_end_trans = 7;
constexpr int NUM_WALKING_PHASE = 8;
};  // namespace WKPhase

class WalkingTest : public Test {
   public:
    WalkingTest(RobotSystem*);
    virtual ~WalkingTest();
    virtual void TestInitialization();

   protected:
    int num_step_;
    ValkyrieStateProvider* sp_;
    virtual int _NextPhase(const int& phase);
    virtual void AdditionalUpdate_();
    void _SettingParameter();

    Planner* centroid_planner_;
    CentroidPlannerParameter* centroid_planner_param_;

    Controller* ds_ctrl_;

    Controller* rs_start_trns_ctrl_;
    Controller* rs_end_trns_ctrl_;
    Controller* ls_start_trns_ctrl_;
    Controller* ls_end_trns_ctrl_;

    YAML::Node cfg_;
};
