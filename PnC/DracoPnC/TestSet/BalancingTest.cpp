#include <PnC/DracoPnC/TestSet/TestSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>

BalancingTest::BalancingTest(RobotSystem* robot) : Test(robot) {
    myUtils::pretty_constructor(1, "Balancing Test");
    cfg_ = YAML::LoadFile(THIS_COM"Config/Draco/TEST/BALANCING_TEST.yaml");

    phase_ = BalancingTestPhase::BT_initial_jpos;
    state_list_.clear();

    jpos_target_ctrl_ = new JPosTargetCtrl(robot);
    body_lift_ctrl_ = new DoubleContactTransCtrl(robot);
    com_ctrl_ = new CoMCtrl(robot);

    state_list_.push_back(jpos_target_ctrl_);
    state_list_.push_back(body_lift_ctrl_);
    state_list_.push_back(com_ctrl_);

    _SettingParameter();
}

BalancingTest::~BalancingTest() {
    for(int i(0); i<state_list_.size(); ++i){
        delete state_list_[i];
    }
}

void BalancingTest::TestInitialization() {
    jpos_target_ctrl_->ctrlInitialization(cfg_["control_configuration"]["joint_position_ctrl"]);
    body_lift_ctrl_->ctrlInitialization(cfg_["control_configuration"]["double_contact_trans_ctrl"]);
    com_ctrl_->ctrlInitialization(cfg_["control_configuration"]["com_ctrl"]);
}

int BalancingTest::_NextPhase(const int & phase) {
    int next_phase = phase + 1;
    printf("next phase: %i\n", next_phase);
    if (next_phase == NUM_BT_PHASE) {
        return BalancingTestPhase::BT_com_ctrl;
    }
    else return next_phase;
}

void BalancingTest::_SettingParameter() {
    try {
        YAML::Node test_cfg = cfg_["test_configuration"];
        Eigen::VectorXd tmp_vec;
        double tmp_val;

        myUtils::readParameter(test_cfg, "initial_jpos", tmp_vec);
        ((JPosTargetCtrl*)jpos_target_ctrl_)->setTargetPosition(tmp_vec);

        myUtils::readParameter(test_cfg, "body_height", tmp_val);
        ((DoubleContactTransCtrl*)body_lift_ctrl_)->setStanceHeight(tmp_val);

        myUtils::readParameter(test_cfg, "com_height", tmp_val);
        ((CoMCtrl*)com_ctrl_)->setCoMHeight(tmp_val);

        myUtils::readParameter(test_cfg, "stabilization_duration", tmp_val);
        ((CoMCtrl*)com_ctrl_)->SetStabilizationDuration(tmp_val);

        myUtils::readParameter(test_cfg, "jpos_initialization_time", tmp_val);
        ((JPosTargetCtrl*)jpos_target_ctrl_)->setMovingTime(tmp_val);

        myUtils::readParameter(test_cfg, "body_lifting_time", tmp_val);
        ((DoubleContactTransCtrl*)body_lift_ctrl_)->setStanceTime(tmp_val);

        myUtils::readParameter(test_cfg, "com_ctrl_time", tmp_val);
        ((CoMCtrl*)com_ctrl_)->setStanceTime(tmp_val);

    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
        exit(0);
    }
}
