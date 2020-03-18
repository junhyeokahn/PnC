#pragma once

#include <PnC/Test.hpp>

class RobotSystem;
class DracoStateProvider;
class WalkingReferenceTrajectoryModule;

enum DCMPhaseWalkingTestPhase{
    DCMPhaseWalkingTestPhase_initial_jpos = 0,
    DCMPhaseWalkingTestPhase_stand_up_ctrl = 1,
    DCMPhaseWalkingTestPhase_double_support_ctrl_1 = 2,
    DCMPhaseWalkingTestPhase_right_swing_start_ctrl = 3,
    DCMPhaseWalkingTestPhase_right_swing_ctrl = 4,
    DCMPhaseWalkingTestPhase_right_swing_end_ctrl = 5,
    DCMPhaseWalkingTestPhase_double_support_ctrl_2 = 6,
    DCMPhaseWalkingTestPhase_left_swing_start_ctrl = 7,
    DCMPhaseWalkingTestPhase_left_swing_ctrl = 8,
    DCMPhaseWalkingTestPhase_left_swing_end_ctrl = 9,
    NUM_DCMPhaseWalkingTestPhase = 10
};

class DCMPhaseWalkingTest: public Test{
    public:
        DCMPhaseWalkingTest(RobotSystem* );
        virtual ~DCMPhaseWalkingTest();
        virtual void TestInitialization();

    protected:
        virtual int _NextPhase(const int & phase);
        void _SettingParameter();

        Controller* jpos_target_ctrl_;
        Controller* stand_up_ctrl_;

        Controller* ds_ctrl_;

        Controller* left_swing_ctrl_;
        Controller* right_swing_ctrl_;

        Controller* right_swing_start_ctrl_;
        Controller* right_swing_end_ctrl_;
        Controller* left_swing_start_ctrl_;
        Controller* left_swing_end_ctrl_;

        WalkingReferenceTrajectoryModule* reference_trajectory_module_;

        YAML::Node cfg_;
        DracoStateProvider* sp_;

        int num_step_;
};
