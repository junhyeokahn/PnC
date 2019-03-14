#include <PnC/FixedAtlasPnC/CtrlSet/JPosTargetCtrl.hpp>
#include <PnC/FixedAtlasPnC/CtrlSet/JPosSwingCtrl.hpp>
#include <PnC/FixedAtlasPnC/TestSet/JointTest.hpp>

JointTest::JointTest(RobotSystem* robot) : Test(robot) {
    myUtils::pretty_constructor(1, "Joint Test");
    cfg_ = YAML::LoadFile(THIS_COM "Config/FixedAtlas/TEST/JOINT_TEST.yaml");

    phase_ = 0;
    state_list_.clear();

    jpos_ctrl_ = new JPosTargetCtrl(robot);
    jpos_swing_ctrl_ = new JPosSwingCtrl(robot);

    state_list_.push_back(jpos_ctrl_);
    state_list_.push_back(jpos_swing_ctrl_);

    _ParameterSetting();
}
JointTest::~JointTest() {
    delete jpos_ctrl_;
    delete jpos_swing_ctrl_;
}

void JointTest::TestInitialization() {
    jpos_ctrl_->ctrlInitialization(
        cfg_["control_configuration"]["joint_position_ctrl"]);
    jpos_swing_ctrl_->ctrlInitialization(
        cfg_["control_configuration"]["joint_swing_ctrl"]);
}
//TODO:: Ask this
int JointTest::_NextPhase(const int& phase) {
    int nx_phase = phase + 1;
    printf("[JPos TEST] next phase: %d\n", nx_phase);
    if (phase == NUM_JPOS_TEST) {
        nx_phase = JPOS_TEST_SWING;
    }
    return nx_phase;
}

void JointTest::_ParameterSetting() {
    try {
        YAML::Node test_cfg = cfg_["test_configuration"];

        Eigen::VectorXd tmp_vec;
        double tmp_val;
        myUtils::readParameter(test_cfg, "target_duration", tmp_val);
        ((JPosTargetCtrl*)jpos_ctrl_)->setEndTime(tmp_val);
        ((JPosTargetCtrl*)jpos_ctrl_)->setMovingTime(tmp_val);
        myUtils::readParameter(test_cfg, "joint_target_pos", tmp_vec);
        ((JPosTargetCtrl*)jpos_ctrl_)->setTargetPosition(tmp_vec);
        ((JPosSwingCtrl*)jpos_swing_ctrl_)->setInitialPosition(tmp_vec);
        myUtils::readParameter(test_cfg, "swing_duration", tmp_val);
        ((JPosSwingCtrl*)jpos_swing_ctrl_)->setEndTime(tmp_val);
        myUtils::readParameter(test_cfg, "amplitude", tmp_vec);
        ((JPosSwingCtrl*)jpos_swing_ctrl_)->setAmplitude(tmp_vec);
        myUtils::readParameter(test_cfg, "frequency", tmp_vec);
        ((JPosSwingCtrl*)jpos_swing_ctrl_)->setFrequency(tmp_vec);
        myUtils::readParameter(test_cfg, "phase", tmp_vec);
        ((JPosSwingCtrl*)jpos_swing_ctrl_)->setPhase(tmp_vec);

    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}
