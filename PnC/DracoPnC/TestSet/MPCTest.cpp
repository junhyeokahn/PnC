#include <PnC/DracoPnC/TestSet/TestSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>


MPCTest::MPCTest(RobotSystem* robot) : Test(robot) {
    myUtils::pretty_constructor(1, "MPC Test");
    cfg_ = YAML::LoadFile(THIS_COM"Config/Draco/TEST/MPC_TEST.yaml");

    phase_ = MPCTestPhase::MPCT_initial_jpos;
    // phase_  = MPCTestPhase::MPCT_force_ctrl;
    state_list_.clear();

    jpos_target_ctrl_ = new IVDJPosTargetCtrl(robot);
    mpc_ctrl_ = new MPCBalanceCtrl(robot);

    state_list_.push_back(jpos_target_ctrl_);
    state_list_.push_back(mpc_ctrl_);

    sp_ = DracoStateProvider::getStateProvider(robot_);
    _SettingParameter();
}

MPCTest::~MPCTest() {
    for(int i(0); i<state_list_.size(); ++i){
        delete state_list_[i];
    }
}

void MPCTest::TestInitialization() {
    jpos_target_ctrl_->ctrlInitialization(cfg_["control_configuration"]["joint_position_ctrl"]);
    mpc_ctrl_->ctrlInitialization(cfg_["control_configuration"]["mpc_ctrl"]);
}

int MPCTest::_NextPhase(const int & phase) {
    int next_phase = phase + 1;
    printf("next phase: %i\n", next_phase);
    if (next_phase == NUM_MPCT_PHASE) {
        return MPCTestPhase::MPCT_force_ctrl;
    }
    else return next_phase;
}

void MPCTest::_SettingParameter() {
    try {
        YAML::Node test_cfg = cfg_["test_configuration"];
        Eigen::VectorXd tmp_vec;
        double tmp_val;

        myUtils::readParameter(test_cfg, "initial_jpos", tmp_vec);
        ((IVDJPosTargetCtrl*)jpos_target_ctrl_)->setTargetPosition(tmp_vec);

        myUtils::readParameter(test_cfg, "jpos_initialization_time", tmp_val);
        ((IVDJPosTargetCtrl*)jpos_target_ctrl_)->setMovingTime(tmp_val);

        myUtils::readParameter(test_cfg, "jpos_ctrl_time", tmp_val);
        ((IVDJPosTargetCtrl*)jpos_target_ctrl_)->setTotalCtrlTime(tmp_val);

        myUtils::readParameter(test_cfg, "com_height", tmp_val);
        ((MPCBalanceCtrl*)mpc_ctrl_)->setCoMHeight(tmp_val);
        sp_-> omega = sqrt(9.81/tmp_val);
        
        myUtils::readParameter(test_cfg, "stabilization_duration", tmp_val);
        ((MPCBalanceCtrl*)mpc_ctrl_)->SetStabilizationDuration(tmp_val);

        myUtils::readParameter(test_cfg, "com_ctrl_time", tmp_val);
        ((MPCBalanceCtrl*)mpc_ctrl_)->setStanceTime(tmp_val);

        myUtils::readParameter(test_cfg, "sway_start_time", tmp_val);
        ((MPCBalanceCtrl*)mpc_ctrl_)->setSwayStartTime(tmp_val);
        myUtils::readParameter(test_cfg, "sway_magnitude", tmp_val);
        ((MPCBalanceCtrl*)mpc_ctrl_)->setSwayMagnitude(tmp_val);
        myUtils::readParameter(test_cfg, "sway_period", tmp_val);
        ((MPCBalanceCtrl*)mpc_ctrl_)->setSwayPeriod(tmp_val);

    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
        exit(0);
    }
}