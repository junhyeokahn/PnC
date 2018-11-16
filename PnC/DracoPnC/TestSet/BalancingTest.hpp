#pragma once

#include <PnC/Test.hpp>

class RobotSystem;

enum BalancingTestPhase{
    BT_initial_jpos = 0,
    BT_lift_up = 1,
    BT_body_ctrl = 2,
    NUM_BT_PHASE
};

class BalancingTest: public Test{
    public:
        BalancingTest(RobotSystem* );
        virtual ~BalancingTest();
        virtual void TestInitialization();

    protected:
        virtual int _NextPhase(const int & phase);
        void _SettingParameter();

        Controller* jpos_target_ctrl_;
        Controller* body_lift_ctrl_;
        Controller* balancing_ctrl_;
};
