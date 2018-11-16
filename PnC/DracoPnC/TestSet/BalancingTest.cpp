#include <PnC/DracoPnC/TestSet/TestSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <RobotSystem/RobotSystem.hpp>

BalancingTest::BalancingTest(RobotSystem* robot) : Test(robot) {

    phase_ = BodyCtrlPhase::BC_initial_jpos;
    state_list_.clear();

    jpos_target_ctrl_ = new JPosTargetCtrl(robot);
    body_lift_ctrl_ = new DoubleContactTransCtrl(robot);
    balancing_ctrl_ = new BalancingCtrl(robot);

    state_list_.push_back(jpos_target_ctrl_);
    state_list_.push_back(body_lift_ctrl_);
    state_list_.push_back(balancing_ctrl_);

    _SettingParameter();

    printf("[Balancing Test] Constructed\n");
}

BalancingTest::~BalancingTest() {
    for(int i(0); i<state_list_.size(); ++i){
        delete state_list_[i];
    }
}

void BalancingTest::TestInitialization() {
    // Yaml file name
    jpos_target_ctrl_->ctrlInitialization("JOINT_CTRL");
    body_lift_ctrl_->ctrlInitialization("DOUBLE_CONTACT_TRANS_CTRL");
    balancing_ctrl_->ctrlInitialization("BALANCING_CTRL");
}

int BalancingTest::_NextPhase(const int & phase) {
    int next_phase = phase + 1;
    printf("next phase: %i\n", next_phase);
    if (next_phase == NUM_BC_PHASE) {
        return BodyCtrlPhase::BC_body_ctrl;
    }
    else return next_phase;
}

void BalancingTest::_SettingParameter() {
    try {
        YAML::Node cfg = YAML::LoadFile(THIS_COM"Config/Draco/TEST/BALANCING_TEST.yaml");
        Eigen::VectorXd tmp_vec;
        double tmp_val;

        myUtils::readParameter(cfg, "initial_jpos", tmp_vec);
        ((JPosTargetCtrl*)jpos_target_ctrl_)->setTargetPosition(tmp_vec);

        myUtils::readParameter(cfg, "body_height", tmp_val);
        ((DoubleContactTransCtrl*)body_lift_ctrl_)->setStanceHeight(tmp_val);

        myUtils::readParameter(cfg, "jpos_initialization_time", tmp_val);
        ((JPosTargetCtrl*)jpos_target_ctrl_)->setMovingTime(tmp_val);
        myUtils::readParameter(cfg, "body_lifting_time", tmp_val);
        ((DoubleContactTransCtrl*)body_lift_ctrl_)->setStanceTime(tmp_val);

        myUtils::readParameter(cfg, "balancing_ctrl_time", tmp_val);
        ((BalancingCtrl*)balancing_ctrl_)->setBalancingTime(tmp_val);
        myUtils::readParameter(cfg, "interpolation_time", tmp_val);
        ((BalancingCtrl*)balancing_ctrl_)->setInterpolationTime(tmp_val);

    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
        exit(0);
    }
}
