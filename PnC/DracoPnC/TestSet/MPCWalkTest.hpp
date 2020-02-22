#pragma once

#include <PnC/Test.hpp>

class RobotSystem;
class DracoStateProvider;

enum MPCWalkTestPhase{
    MPCWalkTestPhase_initial_jpos = 0,
    MPCWalkTestPhase_force_ctrl = 1,
    NUM_MPCWalkTestPhase
};

class MPCWalkTest: public Test{
    public:
        MPCWalkTest(RobotSystem* );
        virtual ~MPCWalkTest();
        virtual void TestInitialization();

    protected:
        virtual int _NextPhase(const int & phase);
        void _SettingParameter();

        Controller* jpos_target_ctrl_;
        Controller* mpc_ctrl_;

        YAML::Node cfg_;
        DracoStateProvider* sp_;
};
