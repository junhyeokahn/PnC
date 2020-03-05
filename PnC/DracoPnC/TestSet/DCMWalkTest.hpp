#pragma once

#include <PnC/Test.hpp>

class RobotSystem;
class DracoStateProvider;

enum DCMWalkTestPhase{
    DCMWalkTestPhase_initial_jpos = 0,
    DCMWalkTestPhase_force_ctrl = 1,
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

        Controller* jpos_target_ctrl_;
        Controller* dcm_ctrl_;
        Controller* dcm_stand_ctrl_;

        YAML::Node cfg_;
        DracoStateProvider* sp_;
};
