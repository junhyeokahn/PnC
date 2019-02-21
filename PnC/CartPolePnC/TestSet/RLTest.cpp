#include <PnC/CartPolePnC/CtrlSet/LearningCtrl.hpp>
#include <PnC/CartPolePnC/CtrlSet/PolicyCtrl.hpp>
#include <PnC/CartPolePnC/TestSet/RLTest.hpp>

RLTest::RLTest(RobotSystem* robot, int mpi_idx, int env_idx) : Test(robot) {
    myUtils::pretty_constructor(1, "RL Test");
    cfg_ = YAML::LoadFile(THIS_COM "Config/CartPole/TEST/RL_TEST.yaml");

    phase_ = 0;
    state_list_.clear();

    learning_ctrl_ = new LearningCtrl(robot, mpi_idx, env_idx);
    b_learning_ = true;

    state_list_.push_back(learning_ctrl_);

    _ParameterSetting();
}

RLTest::RLTest(RobotSystem* robot) : Test(robot) {
    myUtils::pretty_constructor(1, "RL Test");
    cfg_ = YAML::LoadFile(THIS_COM "Config/CartPole/TEST/RL_TEST.yaml");

    phase_ = 0;
    state_list_.clear();

    policy_ctrl_ = new PolicyCtrl(robot);
    b_learning_ = false;

    state_list_.push_back(policy_ctrl_);

    _ParameterSetting();
}

RLTest::~RLTest() { delete learning_ctrl_; }

void RLTest::TestInitialization() {
    if (b_learning_) {
        learning_ctrl_->ctrlInitialization(
            cfg_["control_configuration"]["learning_ctrl"]);
    } else {
        policy_ctrl_->ctrlInitialization(
            cfg_["control_configuration"]["policy_ctrl"]);
    }
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
        if (b_learning_) {
            myUtils::readParameter(test_cfg, "duration", tmp_val);
            ((LearningCtrl*)learning_ctrl_)->setDuration(tmp_val);
            myUtils::readParameter(test_cfg, "obs_lower_bound", tmp_vec);
            ((LearningCtrl*)learning_ctrl_)->setObsLowerBound(tmp_vec);
            myUtils::readParameter(test_cfg, "obs_upper_bound", tmp_vec);
            ((LearningCtrl*)learning_ctrl_)->setObsUpperBound(tmp_vec);
            myUtils::readParameter(test_cfg, "terminate_obs_lower_bound",
                                   tmp_vec);
            ((LearningCtrl*)learning_ctrl_)->setTerminateObsLowerBound(tmp_vec);
            myUtils::readParameter(test_cfg, "terminate_obs_upper_bound",
                                   tmp_vec);
            ((LearningCtrl*)learning_ctrl_)->setTerminateObsUpperBound(tmp_vec);
            myUtils::readParameter(test_cfg, "action_lower_bound", tmp_vec);
            ((LearningCtrl*)learning_ctrl_)->setActLowerBound(tmp_vec);
            myUtils::readParameter(test_cfg, "action_upper_bound", tmp_vec);
            ((LearningCtrl*)learning_ctrl_)->setActUpperBound(tmp_vec);
            myUtils::readParameter(test_cfg, "action_scale", tmp_val);
            ((LearningCtrl*)learning_ctrl_)->setActScale(tmp_val);
        } else {
            myUtils::readParameter(test_cfg, "duration", tmp_val);
            ((PolicyCtrl*)policy_ctrl_)->setDuration(tmp_val);
            myUtils::readParameter(test_cfg, "obs_lower_bound", tmp_vec);
            ((PolicyCtrl*)policy_ctrl_)->setObsLowerBound(tmp_vec);
            myUtils::readParameter(test_cfg, "obs_upper_bound", tmp_vec);
            ((PolicyCtrl*)policy_ctrl_)->setObsUpperBound(tmp_vec);
            myUtils::readParameter(test_cfg, "terminate_obs_lower_bound",
                                   tmp_vec);
            ((PolicyCtrl*)policy_ctrl_)->setTerminateObsLowerBound(tmp_vec);
            myUtils::readParameter(test_cfg, "terminate_obs_upper_bound",
                                   tmp_vec);
            ((PolicyCtrl*)policy_ctrl_)->setTerminateObsUpperBound(tmp_vec);
            myUtils::readParameter(test_cfg, "action_lower_bound", tmp_vec);
            ((PolicyCtrl*)policy_ctrl_)->setActLowerBound(tmp_vec);
            myUtils::readParameter(test_cfg, "action_upper_bound", tmp_vec);
            ((PolicyCtrl*)policy_ctrl_)->setActUpperBound(tmp_vec);
            myUtils::readParameter(test_cfg, "action_scale", tmp_val);
            ((PolicyCtrl*)policy_ctrl_)->setActScale(tmp_val);
        }

    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}
