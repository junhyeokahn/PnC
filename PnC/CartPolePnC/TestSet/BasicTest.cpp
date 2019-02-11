#include <PnC/CartPolePnC/TestSet/BasicTest.hpp>
#include <PnC/CartPolePnC/CtrlSet/BasicCtrl.hpp>

BasicTest::BasicTest(RobotSystem* robot) : Test(robot){
    myUtils::pretty_constructor(1, "Basic Test");
    cfg_ = YAML::LoadFile(THIS_COM"Config/CartPole/TEST/BASIC_TEST.yaml");

    phase_ = 0;
    state_list_.clear();

    basic_ctrl_ = new BasicCtrl(robot);

    state_list_.push_back(basic_ctrl_);

    _ParameterSetting();
}
BasicTest::~BasicTest(){
    delete basic_ctrl_;
}

void BasicTest::TestInitialization(){
}

int BasicTest::_NextPhase(const int & phase) {
    int nx_phase = phase + 1;
    printf("[BASIC TEST] next phase: %d\n", nx_phase);
    if(phase == NUM_BASIC_TEST){
        nx_phase = BASIC;
    }
    return nx_phase;
}

void BasicTest::_ParameterSetting(){
    try {
        YAML::Node test_cfg = cfg_["test_configuration"];

        Eigen::VectorXd tmp_vec;
        double tmp_val;
        myUtils::readParameter(test_cfg, "duration", tmp_val);
        ((BasicCtrl*)basic_ctrl_)->setDuration(tmp_val);

    }catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
        exit(0);
    }

}
