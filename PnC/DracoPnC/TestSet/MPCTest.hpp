#pragma once

#include <PnC/Test.hpp>

class RobotSystem;
class DracoStateProvider;

enum MPCTestPhase{
    MPCT_initial_jpos = 0,
    // MPCT_force_ctrl = 1,
    NUM_MPCT_PHASE
};

class MPCTest: public Test{
    public:
        MPCTest(RobotSystem* );
        virtual ~MPCTest();
        virtual void TestInitialization();

    protected:
        virtual int _NextPhase(const int & phase);
        void _SettingParameter();

        Controller* jpos_target_ctrl_;

        YAML::Node cfg_;
        DracoStateProvider* sp_;
};
