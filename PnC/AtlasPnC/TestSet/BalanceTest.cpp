#include <PnC/AtlasPnC/AtlasDefinition.hpp>
#include <PnC/AtlasPnC/AtlasStateProvider.hpp>
#include <PnC/AtlasPnC/ContactSet/ContactSet.hpp>
#include <PnC/AtlasPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/AtlasPnC/TaskSet/TaskSet.hpp>
#include <PnC/AtlasPnC/TestSet/BalanceTest.hpp>
#include <Utils/IO/DataManager.hpp>

BalanceTest::BalanceTest(RobotSystem* robot) : Test(robot) {
    myUtils::pretty_constructor(1, "Balance Test");
    cfg_ = YAML::LoadFile(THIS_COM "Config/Atlas/TEST/BALANCE_TEST.yaml");

    sp_ = AtlasStateProvider::getStateProvider(robot_);
    phase_ = BLPhase::BALANCE;

    state_list_.clear();

    balance_ctrl_ = new BalanceCtrl(robot);
    _SettingParameter();

    state_list_.push_back(balance_ctrl_);
}

BalanceTest::~BalanceTest() { delete balance_ctrl_; }

void BalanceTest::TestInitialization() {
    balance_ctrl_->ctrlInitialization(
        cfg_["control_configuration"]["balance_ctrl"]);
}

int BalanceTest::_NextPhase(const int& phase) {
    // int next_phase = phase + 1;
    return 0;
}

void BalanceTest::_SettingParameter() {}

void BalanceTest::AdditionalUpdate_() {}
