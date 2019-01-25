#include <PnC/DracoPnC/TestSet/TestSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>

JointTest::JointTest(RobotSystem* robot) : Test(robot){
    myUtils::pretty_constructor(1, "Joint Test");
    cfg_ = YAML::LoadFile(THIS_COM"Config/Draco/TEST/JOINT_TEST.yaml");

    phase_ = 0;
    state_list_.clear();

    jpos_target_ctrl_ = new JPosTargetCtrl(robot);
    jpos_swing_ctrl_ = new JPosSwingCtrl(robot);

    state_list_.push_back(jpos_target_ctrl_);
    state_list_.push_back(jpos_swing_ctrl_);

    _ParameterSetting();
}
JointTest::~JointTest(){
    delete jpos_target_ctrl_;
    delete jpos_swing_ctrl_;
}

void JointTest::TestInitialization(){
    jpos_target_ctrl_->ctrlInitialization(cfg_["control_configuration"]["joint_position_ctrl"]);
    jpos_swing_ctrl_->ctrlInitialization(cfg_["control_configuration"]["joint_swing_ctrl"]);
    std::cout << "[Joint Test] Initialized" << std::endl;
}

int JointTest::_NextPhase(const int & phase){
    int nx_phase = phase + 1;
    printf("[JPos TEST] next phase: %d\n", nx_phase);
    if(phase == NUM_JPOS_TEST){
        nx_phase = JPOS_TEST_SWING;
    }
    return nx_phase;
}

void JointTest::_ParameterSetting(){
    try {
        YAML::Node test_cfg = cfg_["test_configuration"];

        Eigen::VectorXd tmp_vec;
        double tmp_val;
        myUtils::readParameter(test_cfg, "initial_jpos", tmp_vec);
        ((JPosTargetCtrl*)jpos_target_ctrl_)->setTargetPosition(tmp_vec);
        ((JPosSwingCtrl*)jpos_swing_ctrl_)->setPosture(tmp_vec); // swing test initial posture set
        myUtils::readParameter(test_cfg, "initialization_time", tmp_val);
        ((JPosTargetCtrl*)jpos_target_ctrl_)->setMovingTime(tmp_val);
        myUtils::readParameter(test_cfg, "test_duration", tmp_val);
        ((JPosSwingCtrl*)jpos_swing_ctrl_)->setMovingTime(tmp_val);
        myUtils::readParameter(test_cfg, "amplitude", tmp_vec);
        ((JPosSwingCtrl*)jpos_swing_ctrl_)->setAmplitude(tmp_vec);
        myUtils::readParameter(test_cfg, "frequency", tmp_vec);
        ((JPosSwingCtrl*)jpos_swing_ctrl_)->setFrequency(tmp_vec);
        myUtils::readParameter(test_cfg, "phase", tmp_vec);
        ((JPosSwingCtrl*)jpos_swing_ctrl_)->setPhase(tmp_vec);

    }catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
        exit(0);
    }

}
