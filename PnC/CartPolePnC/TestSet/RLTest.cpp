#include <PnC/CartPolePnC/CtrlSet/NeuralNetCtrl.hpp>
#include <PnC/CartPolePnC/TestSet/RLTest.hpp>

RLTest::RLTest(RobotSystem* robot) : Test(robot) {
    myUtils::pretty_constructor(1, "RL Test");
    cfg_ = YAML::LoadFile(THIS_COM "Config/CartPole/TEST/RL_TEST.yaml");

    phase_ = 0;
    state_list_.clear();

    nn_ctrl_ = new NeuralNetCtrl(robot);

    state_list_.push_back(nn_ctrl_);

    _ParameterSetting();
}
RLTest::~RLTest() { delete nn_ctrl_; }

void RLTest::TestInitialization() {
    nn_ctrl_->ctrlInitialization(cfg_["control_configuration"]["nn_ctrl"]);
}

int RLTest::_NextPhase(const int& phase) {
    int nx_phase = phase + 1;
    printf("[RL TEST] next phase: %d\n", nx_phase);
    if (phase == NUM_RL_TEST) {
        nx_phase = NN;
    }
    return nx_phase;
}

void RLTest::_ParameterSetting() {
    try {
        YAML::Node test_cfg = cfg_["test_configuration"];

        Eigen::VectorXd tmp_vec;
        double tmp_val;
        myUtils::readParameter(test_cfg, "duration", tmp_val);
        ((NeuralNetCtrl*)nn_ctrl_)->setDuration(tmp_val);

    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}
