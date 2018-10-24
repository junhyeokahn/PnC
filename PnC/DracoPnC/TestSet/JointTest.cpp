#include <PnC/DracoPnC/TestSet/TestSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>

JointTest::JointTest(RobotSystem* robot) : Test(robot){
    phase_ = 0;
    state_list_.clear();

    jpos_target_ctrl_ = new JPosTargetCtrl(robot);
    jpos_swing_ctrl_ = new JPosSwingCtrl(robot);

    state_list_.push_back(jpos_target_ctrl_);
    state_list_.push_back(jpos_swing_ctrl_);

    _ParameterSetting();
    printf("[Joint Test] Constructed\n");
}
JointTest::~JointTest(){
    delete jpos_target_ctrl_;
    delete jpos_swing_ctrl_;
}

void JointTest::TestInitialization(){
    jpos_target_ctrl_->ctrlInitialization("JOINT_TARGET_CTRL");
    jpos_swing_ctrl_->ctrlInitialization("JOINT_SWING_CTRL");
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
        YAML::Node cfg = YAML::LoadFile(THIS_COM"Config/Draco/TEST/JOINT_TEST.yaml");
        Eigen::VectorXd tmp_vec;
        double tmp_val;
        myUtils::readParameter(cfg, "initial_jpos", tmp_vec);
        ((JPosTargetCtrl*)jpos_target_ctrl_)->setTargetPosition(tmp_vec);
        ((JPosSwingCtrl*)jpos_swing_ctrl_)->setPosture(tmp_vec); // swing test initial posture set
        myUtils::readParameter(cfg, "initialization_time", tmp_val);
        ((JPosTargetCtrl*)jpos_target_ctrl_)->setMovingTime(tmp_val);
        myUtils::readParameter(cfg, "test_duration", tmp_val);
        ((JPosSwingCtrl*)jpos_swing_ctrl_)->setMovingTime(tmp_val);
        myUtils::readParameter(cfg, "amplitude", tmp_vec);
        ((JPosSwingCtrl*)jpos_swing_ctrl_)->setAmplitude(tmp_vec);
        myUtils::readParameter(cfg, "frequency", tmp_vec);
        ((JPosSwingCtrl*)jpos_swing_ctrl_)->setFrequency(tmp_vec);
        myUtils::readParameter(cfg, "phase", tmp_vec);
        ((JPosSwingCtrl*)jpos_swing_ctrl_)->setPhase(tmp_vec);
    }catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
        exit(0);
    }

}
