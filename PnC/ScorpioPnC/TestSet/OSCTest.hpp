#pragma once
#include <PnC/Test.hpp>

class RobotSystem;

enum OSC_TEST_PHASE {OSC_POS_TEST = 0};

class OSCTest : public Test {
   public:
    OSCTest(RobotSystem* robot);
    virtual ~OSCTest();

    virtual void TestInitialization();

   protected:
    void _ParameterSetting();
    virtual int _NextPhase(const int& phase);

    Controller* osc_pos_ctrl_;
    //Controller* osc_ori_ctrl_;
    YAML::Node cfg_;
};
