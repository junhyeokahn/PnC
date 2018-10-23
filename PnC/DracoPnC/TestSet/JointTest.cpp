#include <PnC/DracoPnC/TestSet/TestSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>

JointTest::JointTest(RobotSystem* robot) : Test(robot){
    phase_ = 0;
    state_list_.clear();

    jpos_ctrl_ini_ = new JPosTargetCtrl(robot);
    jpos_ctrl_ = new JPosCtrl(robot);

    state_list_.push_back(jpos_ctrl_ini_);
    state_list_.push_back(jpos_ctrl_);

    _ParameterSetting();
    printf("[Joint Test] Constructed\n");
}
JointTest::~JointTest(){
    delete jpos_ctrl_ini_;
    delete jpos_ctrl_;
}

void JointTest::TestInitialization(){
    jpos_ctrl_ini_->ctrlInitialization("JOINT_INITIALIZE_CTRL");
    jpos_ctrl_->ctrlInitialization("JOINT_SWING_CTRL");
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
        ((JPosTargetCtrl*)jpos_ctrl_ini_)->setTargetPosition(tmp_vec);
        ((JPosCtrl*)jpos_ctrl_)->setPosture(tmp_vec); // swing test initial posture set
        myUtils::readParameter(cfg, "initialization_time", tmp_val);
        ((JPosTargetCtrl*)jpos_ctrl_ini_)->setMovingTime(tmp_val);
        myUtils::readParameter(cfg, "test_duration", tmp_val);
        ((JPosCtrl*)jpos_ctrl_)->setMovingTime(tmp_val);
        myUtils::readParameter(cfg, "amplitude", tmp_vec);
        ((JPosCtrl*)jpos_ctrl_)->setAmplitude(tmp_vec);
        myUtils::readParameter(cfg, "frequency", tmp_vec);
        ((JPosCtrl*)jpos_ctrl_)->setFrequency(tmp_vec);
        myUtils::readParameter(cfg, "phase", tmp_vec);
        ((JPosCtrl*)jpos_ctrl_)->setPhase(tmp_vec);
    }catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
        exit(0);
    }

}
