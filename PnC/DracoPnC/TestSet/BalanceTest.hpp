#pragma once

#include <PnC/Test.hpp>

class DracoStateProvider;

namespace BLPhase {
constexpr int BALANCE = 0;
constexpr int SWING = 1;
};  // namespace BLPhase

class BalanceTest : public Test {
   public:
    BalanceTest(RobotSystem*);
    virtual ~BalanceTest();
    virtual void TestInitialization();

   protected:
    int num_step_;
    DracoStateProvider* sp_;
    virtual int _NextPhase(const int& phase);
    virtual void AdditionalUpdate_();
    void _SettingParameter();

    Controller* balance_ctrl_;
    Controller* swing_ctrl_;

    YAML::Node cfg_;
};
