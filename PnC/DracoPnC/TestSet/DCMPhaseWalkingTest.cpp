#include <PnC/DracoPnC/TestSet/TestSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>

DCMPhaseWalkingTest::DCMPhaseWalkingTest(RobotSystem* robot) : Test(robot) {
    myUtils::pretty_constructor(1, "DCM Phase Walking Test");
    cfg_ = YAML::LoadFile(
            THIS_COM"Config/Draco/TEST/DCM_PHASE_WALKING_TEST.yaml");

    num_step_ = 0;
    phase_ = DCMPhaseWalkingTestPhase::DCMPhaseWalkingTestPhase_initial_jpos;
    state_list_.clear();

    jpos_target_ctrl_ = new IVDJPosTargetCtrl(robot);
    stand_ctrl_ = new StandCtrl(robot);

    ds_ctrl_ = new DoubleSupportCtrl(robot);

    right_swing_ctrl_ = new SingleSupportCtrl(robot,
            DCMPhaseWalkingTestPhase::DCMPhaseWalkingTestPhase_right_swing_ctrl);
    left_swing_ctrl_ = new SingleSupportCtrl(robot,
            DCMPhaseWalkingTestPhase::DCMPhaseWalkingTestPhase_left_swing_ctrl);

    right_swing_start_ctrl_ = new TransitionCtrl(robot,
            DCMPhaseWalkingTestPhase::DCMPhaseWalkingTestPhase_right_swing_start_ctrl);
    right_swing_end_ctrl_ = new TransitionCtrl(robot,
            DCMPhaseWalkingTestPhase::DCMPhaseWalkingTestPhase_right_swing_end_ctrl);
    left_swing_start_ctrl_ = new TransitionCtrl(robot,
            DCMPhaseWalkingTestPhase::DCMPhaseWalkingTestPhase_left_swing_start_ctrl);
    left_swing_end_ctrl_ = new TransitionCtrl(robot,
            DCMPhaseWalkingTestPhase::DCMPhaseWalkingTestPhase_left_swing_end_ctrl);

    state_list_.push_back(jpos_target_ctrl_);
    state_list_.push_back(stand_ctrl_);
    state_list_.push_back(ds_ctrl_);
    state_list_.push_back(right_swing_start_ctrl_);
    state_list_.push_back(right_swing_ctrl_);
    state_list_.push_back(right_swing_end_ctrl_);
    state_list_.push_back(ds_ctrl_);
    state_list_.push_back(left_swing_start_ctrl_);
    state_list_.push_back(left_swing_ctrl_);
    state_list_.push_back(left_swing_end_ctrl_);

    sp_ = DracoStateProvider::getStateProvider(robot_);

    _SettingParameter();
}

DCMPhaseWalkingTest::~DCMPhaseWalkingTest() {
    for(int i(0); i<state_list_.size(); ++i){
        delete state_list_[i];
    }
}

void DCMPhaseWalkingTest::TestInitialization() {
    jpos_target_ctrl_->ctrlInitialization(
            cfg_["control_configuration"]["joint_position_ctrl"]);
    stand_ctrl_->ctrlInitialization(
            cfg_["control_configuration"]["stand_ctrl"]);
    ds_ctrl_->ctrlInitialization(
            cfg_["control_configuration"]["ds_ctrl"]);
    right_swing_ctrl_->ctrlInitialization(
            cfg_["control_configuration"]["right_swing_ctrl"]);
    left_swing_ctrl_->ctrlInitialization(
            cfg_["control_configuration"]["left_swing_ctrl"]);
    right_swing_start_ctrl_->ctrlInitialization(
            cfg_["control_configuration"]["right_swing_start_ctrl"]);
    right_swing_end_ctrl_->ctrlInitialization(
            cfg_["control_configuration"]["right_swing_end_ctrl"]);
    left_swing_start_ctrl_->ctrlInitialization(
            cfg_["control_configuration"]["left_swing_start_ctrl"]);
    left_swing_end_ctrl_->ctrlInitialization(
            cfg_["control_configuration"]["left_swing_end_ctrl"]);
}

int DCMPhaseWalkingTest::_NextPhase(const int & phase) {
    int next_phase = phase + 1;

    if (phase == DCMPhaseWalkingTestPhase::DCMPhaseWalkingTestPhase_double_support_ctrl_1) {
        ++num_step_;
        sp_->stance_foot = "lFoot";
    }

    if (phase == DCMPhaseWalkingTestPhase::DCMPhaseWalkingTestPhase_double_support_ctrl_2) {
        ++num_step_;
        sp_->stance_foot = "rFoot";
    }
    sp_->num_step_copy = num_step_;

    if (next_phase == NUM_DCMPhaseWalkingTestPhase) {
        return DCMPhaseWalkingTestPhase::DCMPhaseWalkingTestPhase_double_support_ctrl_1;
    } else {
        return next_phase;
    }
}

void DCMPhaseWalkingTest::_SettingParameter() {
    try {
        YAML::Node test_cfg = cfg_["test_configuration"];
        Eigen::VectorXd tmp_vec;
        double tmp_val;

        // jpos_target_ctrl
        myUtils::readParameter(test_cfg, "initial_jpos", tmp_vec);
        ((IVDJPosTargetCtrl*)jpos_target_ctrl_)->setTargetPosition(tmp_vec);
        myUtils::readParameter(test_cfg, "jpos_initialization_time", tmp_val);
        ((IVDJPosTargetCtrl*)jpos_target_ctrl_)->setMovingTime(tmp_val);
        myUtils::readParameter(test_cfg, "jpos_ctrl_time", tmp_val);
        ((IVDJPosTargetCtrl*)jpos_target_ctrl_)->setTotalCtrlTime(tmp_val);

        // stand_ctrl

        myUtils::readParameter(test_cfg, "com_height", tmp_val);
        ((MPCWalkCtrl*)mpc_ctrl_)->setCoMHeight(tmp_val);
        sp_->omega = sqrt(9.81/tmp_val);
 
        myUtils::readParameter(test_cfg, "contact_transition_duration", tmp_val);
        ((MPCWalkCtrl*)mpc_ctrl_)->setContactTransitionTime(tmp_val);

        myUtils::readParameter(test_cfg, "stabilization_duration", tmp_val);
        ((MPCWalkCtrl*)mpc_ctrl_)->SetStabilizationDuration(tmp_val);

        myUtils::readParameter(test_cfg, "com_ctrl_time", tmp_val);
        ((MPCWalkCtrl*)mpc_ctrl_)->setStanceTime(tmp_val);

        myUtils::readParameter(test_cfg, "walk_start_time", tmp_val);
        ((MPCWalkCtrl*)mpc_ctrl_)->setWalkStartTime(tmp_val);

    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
        exit(0);
    }
}
