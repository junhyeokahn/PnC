#include <PnC/ValkyriePnC/ContactSet/ContactSet.hpp>
#include <PnC/ValkyriePnC/CtrlSet/CtrlSet.hpp>
#include <PnC/ValkyriePnC/TaskSet/TaskSet.hpp>
#include <PnC/ValkyriePnC/TestSet/BalanceTest.hpp>
#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>
#include <Utils/IO/DataManager.hpp>

BalanceTest::BalanceTest(RobotSystem* robot) : Test(robot) {
    myUtils::pretty_constructor(1, "Balance Test");
    cfg_ = YAML::LoadFile(THIS_COM "Config/Valkyrie/TEST/BALANCE_TEST.yaml");

    sp_ = ValkyrieStateProvider::getStateProvider(robot_);
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
