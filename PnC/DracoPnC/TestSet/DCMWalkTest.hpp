#pragma once

#include <PnC/Test.hpp>

class RobotSystem;
class DracoStateProvider;

enum DCMWalkTestPhase{
    DCMWalkTestPhase_initial_jpos = 0,
    DCMWalkTestPhase_stand_ctrl = 1,
    DCMWalkTestPhase_walk_ctrl = 2,
    DCMWalkTestPhase_balance_ctrl = 3,
    NUM_DCMWalkTestPhase
};

class DCMWalkTest: public Test{
    public:
        DCMWalkTest(RobotSystem* );
        virtual ~DCMWalkTest();
        virtual void TestInitialization();

    protected:
        virtual int _NextPhase(const int & phase);
        void _SettingParameter();

        int repeat_counter_;
        int repeat_times_;        

        Controller* jpos_target_ctrl_;
        Controller* dcm_stand_ctrl_;
        Controller* dcm_balance_ctrl_;
        Controller* dcm_ctrl_;

        YAML::Node cfg_;
        DracoStateProvider* sp_;
};
