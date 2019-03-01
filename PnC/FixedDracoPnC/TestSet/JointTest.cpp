#include <PnC/FixedDracoPnC/CtrlSet/JPosTargetCtrl.hpp>
#include <PnC/FixedDracoPnC/TestSet/JointTest.hpp>

JointTest::JointTest(RobotSystem* robot) : Test(robot) {
    myUtils::pretty_constructor(1, "Joint Test");
    cfg_ = YAML::LoadFile(THIS_COM "Config/FixedDraco/TEST/JOINT_TEST.yaml");

    phase_ = 0;
    state_list_.clear();

    jpos_ctrl_ = new JPosTargetCtrl(robot);
    state_list_.push_back(jpos_ctrl_);

    _ParameterSetting();
}
JointTest::~JointTest() { delete jpos_ctrl_; }

void JointTest::TestInitialization() {
    jpos_ctrl_->ctrlInitialization(
        cfg_["control_configuration"]["joint_position_ctrl"]);
}

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
        myUtils::readParameter(test_cfg, "joint_ini_duration", tmp_val);
        ((JPosTargetCtrl*)jpos_ctrl_)->setMovingTime(tmp_val);
        myUtils::readParameter(test_cfg, "ini_phase_duration", tmp_val);
        ((JPosTargetCtrl*)jpos_ctrl_)->setEndTime(tmp_val);
        myUtils::readParameter(test_cfg, "joint_target_pos", tmp_vec);
        ((JPosTargetCtrl*)jpos_ctrl_)->setTargetPosition(tmp_vec);

    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}
