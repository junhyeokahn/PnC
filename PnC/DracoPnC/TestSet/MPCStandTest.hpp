#pragma once

#include <PnC/Test.hpp>

class RobotSystem;
class DracoStateProvider;

enum MPCStandTestPhase{
    MPCStandTestPhase_initial_jpos = 0,
    MPCStandTestPhase_force_ctrl = 1,
    NUM_MPCStandTestPhase
};

class MPCStandTest: public Test{
    public:
        MPCStandTest(RobotSystem* );
        virtual ~MPCStandTest();
        virtual void TestInitialization();

    protected:
        virtual int _NextPhase(const int & phase);
        void _SettingParameter();

        Controller* jpos_target_ctrl_;
        Controller* mpc_ctrl_;

        YAML::Node cfg_;
        DracoStateProvider* sp_;
};
