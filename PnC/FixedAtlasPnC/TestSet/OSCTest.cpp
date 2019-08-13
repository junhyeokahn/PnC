#include <PnC/FixedAtlasPnC/CtrlSet/OSCPosCtrl.hpp>
#include <PnC/FixedAtlasPnC/CtrlSet/OSCOriCtrl.hpp>
#include <PnC/FixedAtlasPnC/TestSet/OSCTest.hpp>

OSCTest::OSCTest(RobotSystem* robot) : Test(robot) {
    myUtils::pretty_constructor(1, "OSC Test");
    cfg_ = YAML::LoadFile(THIS_COM "Config/FixedAtlas/TEST/OSC_TEST.yaml");

    phase_ = 0;
    state_list_.clear();

    //osc_pos_ctrl_ = new OSCPosCtrl(robot);
    osc_ori_ctrl_ = new OSCOriCtrl(robot);

    //state_list_.push_back(osc_pos_ctrl_);
    state_list_.push_back(osc_ori_ctrl_);

    _ParameterSetting();
}
OSCTest::~OSCTest() {
    //delete osc_pos_ctrl_;
    delete osc_ori_ctrl_;
}

void OSCTest::TestInitialization() {
    //osc_pos_ctrl_->ctrlInitialization(
        //cfg_["control_configuration"]["osc_position_ctrl"]);
    osc_ori_ctrl_->ctrlInitialization(
        cfg_["control_configuration"]["osc_orientation_ctrl"]);
}
int OSCTest::_NextPhase(const int& phase) {
    int nx_phase = phase + 1;
    printf("[OSC TEST] next phase: %d\n", nx_phase);
    //if (phase == NUM_OSC_TEST) {
        //nx_phase = OSC_POS_TEST;
    //}
    //return nx_phase;
    return 0;
}

void OSCTest::_ParameterSetting() {
    try {
        YAML::Node test_cfg = cfg_["test_configuration"];

        Eigen::VectorXd tmp_vec;
        double tmp_val;
        //myUtils::readParameter(test_cfg, "xyz_duration", tmp_val);
        //((OSCPosCtrl*)osc_pos_ctrl_)->setEndTime(tmp_val);
        //myUtils::readParameter(test_cfg, "rf_target_pos", tmp_vec);
        //((OSCPosCtrl*)osc_pos_ctrl_)->setTargetPosition(tmp_vec);

        myUtils::readParameter(test_cfg, "ori_duration", tmp_val);
        ((OSCOriCtrl*)osc_ori_ctrl_)->setEndTime(tmp_val);
        myUtils::readParameter(test_cfg, "head_target_pos", tmp_vec);
        ((OSCOriCtrl*)osc_ori_ctrl_)->setTargetPosition(tmp_vec);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}
