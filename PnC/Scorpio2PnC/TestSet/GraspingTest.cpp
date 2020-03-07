#include <PnC/Scorpio2PnC/CtrlSet/OSCCtrl.hpp>
#include <PnC/Scorpio2PnC/CtrlSet/GripperCtrl.hpp>
#include <PnC/Scorpio2PnC/TestSet/GraspingTest.hpp>
#include <PnC/Scorpio2PnC/ScorpioStateProvider.hpp>

Grasping2Test::Grasping2Test(RobotSystem* robot) : Test(robot) {
    myUtils::pretty_constructor(1, "Scorpio2 OSC Test");
    cfg_ = YAML::LoadFile(THIS_COM "Config/Scorpio/TEST/GRASPING_TEST.yaml");

    phase_ = 0;
    state_list_.clear();

    gripper_ctrl_ = new Gripper2Ctrl(robot);
    move_ctrl_ = new OSC2Ctrl(robot);

    state_list_.push_back(gripper_ctrl_);
    state_list_.push_back(move_ctrl_);

    _ParameterSetting();
    sp_ = Scorpio2StateProvider::getStateProvider(robot_);
}
Grasping2Test::~Grasping2Test() {
    delete move_ctrl_;
    delete gripper_ctrl_;
}

void Grasping2Test::TestInitialization() {
    move_ctrl_->ctrlInitialization(
            cfg_["control_configuration"]["osc_ctrl"]);
    gripper_ctrl_->ctrlInitialization(
            cfg_["control_configuration"]["grasping_ctrl"]);
}
int Grasping2Test::_NextPhase(const int& phase) {
    int nx_phase = phase + 1;
    //printf("[GRASPING TEST] next phase: %d\n", nx_phase);
    if (nx_phase == NUM_GRASPING2_PH) {
        nx_phase = HOLD2_PH;
    }
    sp_->phase_copy = nx_phase;
    return nx_phase;
}

void Grasping2Test::_ParameterSetting() {
    try {
        YAML::Node test_cfg = cfg_["test_configuration"];

        Eigen::VectorXd tmp_vec;
        double tmp_val;
        myUtils::readParameter(test_cfg, "moving_duration", tmp_val);
        ((OSC2Ctrl*)move_ctrl_)->setEndTime(tmp_val);

    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
            << __FILE__ << "]" << std::endl
            << std::endl;
        exit(0);
    }
}

void Grasping2Test::SetMovingTarget(Eigen::VectorXd pos){
    ((OSC2Ctrl*)move_ctrl_)->setRelativeTargetPosition(pos);
}
