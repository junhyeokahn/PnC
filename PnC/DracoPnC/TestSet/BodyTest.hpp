#pragma once

#include <PnC/Test.hpp>

class RobotSystem;

enum BodyCtrlPhase{
    BC_initial_jpos = 0,
    BC_lift_up = 1,
    BC_body_ctrl = 2,
    NUM_BC_PHASE
};

class BodyTest: public Test{
    public:
        BodyTest(RobotSystem* );
        virtual ~BodyTest();
        virtual void TestInitialization();

    protected:
        virtual int _NextPhase(const int & phase);
        void _SettingParameter();

        Controller* jpos_target_ctrl_;
        Controller* body_lift_ctrl_;
        Controller* body_rpz_ctrl_;
};
