#include <PnC/CartPolePnC/TestSet/PolicyTest.hpp>
#include <PnC/CartPolePnC/CtrlSet/PolicyCtrl.hpp>

PolicyTest::PolicyTest(RobotSystem* robot) : Test(robot){
    myUtils::pretty_constructor(1, "Policy Test");
    cfg_ = YAML::LoadFile(THIS_COM"Config/CartPole/TEST/POLICY_TEST.yaml");

    phase_ = 0;
    state_list_.clear();

    policy_ctrl_ = new PolicyCtrl(robot);

    state_list_.push_back(policy_ctrl_);

    _ParameterSetting();
}
PolicyTest::~PolicyTest(){
    delete policy_ctrl_;
}

void PolicyTest::TestInitialization(){
}

int PolicyTest::_NextPhase(const int & phase) {
    int nx_phase = phase + 1;
    printf("[POLICY TEST] next phase: %d\n", nx_phase);
    if(phase == NUM_POLICY_TEST) {
        nx_phase = POLICY;
    }
    return nx_phase;
}

void PolicyTest::_ParameterSetting() {
    try {
        YAML::Node test_cfg = cfg_["test_configuration"];

        Eigen::VectorXd tmp_vec;
        double tmp_val;
        myUtils::readParameter(test_cfg, "duration", tmp_val);
        ((PolicyCtrl*)policy_ctrl_)->setDuration(tmp_val);
        std::string model_path;
        myUtils::readParameter(test_cfg, "model_path", model_path);
        ((PolicyCtrl*)policy_ctrl_)->setModelPath(model_path);

    }catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
        exit(0);
    }
}
