#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/DracoPnC/TestSet/BalanceTest.hpp>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <Utils/IO/DataManager.hpp>

BalanceTest::BalanceTest(RobotSystem* robot) : Test(robot) {
    myUtils::pretty_constructor(1, "Balance Test");
    cfg_ = YAML::LoadFile(THIS_COM "Config/Draco/TEST/BALANCE_TEST.yaml");

    sp_ = DracoStateProvider::getStateProvider(robot_);
    phase_ = BLPhase::BALANCE;

    state_list_.clear();

    balance_ctrl_ = new BalanceCtrl(robot);
    //swing_ctrl_ = new SwingCtrl(robot);

    _SettingParameter();

    state_list_.push_back(balance_ctrl_);
    //state_list_.push_back(swing_ctrl_);
}

BalanceTest::~BalanceTest() { 
    delete balance_ctrl_; 
    //delete swing_ctrl_; 
}

void BalanceTest::TestInitialization() {
    balance_ctrl_->ctrlInitialization(
        cfg_["control_configuration"]["balance_ctrl"]);
    //swing_ctrl_->ctrlInitialization(
        //cfg_["control_configuration"]["swing_ctrl"]);
}

int BalanceTest::_NextPhase(const int& phase) {
    int next_phase = phase + 1;
    return next_phase;
}

void BalanceTest::_SettingParameter() {
    try {
        double temp;
        Eigen::VectorXd temp_vec;

        YAML::Node test_cfg = cfg_["test_configuration"];
        myUtils::readParameter(test_cfg,"target_pos_duration",temp);
       ((BalanceCtrl*)balance_ctrl_)->setDuration(temp);
        myUtils::readParameter(test_cfg,"com_pos_deviation",temp_vec);
       ((BalanceCtrl*)balance_ctrl_)->setComDeviation(temp_vec);
        //myUtils::readParameter(test_cfg,"amplitude",temp_vec);
       //((SwingCtrl*)swing_ctrl_)->setAmplitude(temp_vec);
        //myUtils::readParameter(test_cfg,"frequency",temp_vec);
       //((SwingCtrl*)swing_ctrl_)->setFrequency(temp_vec);
        //myUtils::readParameter(test_cfg,"phase",temp_vec);
       //((SwingCtrl*)swing_ctrl_)->setPhase(temp_vec);

    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}

void BalanceTest::AdditionalUpdate_() {}
