#pragma once

#include <PnC/Test.hpp>

class AtlasStateProvider;
class Planner;
class CentroidPlannerParameter;

namespace WPGPhase {
constexpr int double_contact_1 = 0;
constexpr int right_swing_start_trans = 1;
constexpr int right_swing = 2;
constexpr int right_swing_end_trans = 3;
constexpr int double_contact_2 = 4;
constexpr int left_swing_start_trans = 5;
constexpr int left_swing = 6;
constexpr int left_swing_end_trans = 7;
constexpr int NUM_WALKING_PHASE = 8;
};  // namespace WPGPhase

class WPGTest : public Test {
   public:
    WPGTest(RobotSystem*);
    virtual ~WPGTest();
    virtual void TestInitialization();

   protected:
    int num_step_;
    AtlasStateProvider* sp_;
    virtual int _NextPhase(const int& phase);
    virtual void AdditionalUpdate_();
    void _SettingParameter();

    Planner* centroid_planner_;
    CentroidPlannerParameter* centroid_planner_param_;

    Controller* ds_ctrl_;

    YAML::Node cfg_;
};
