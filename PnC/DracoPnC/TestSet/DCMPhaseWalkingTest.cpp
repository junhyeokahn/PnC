#include <PnC/DracoPnC/TestSet/TestSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/PredictionModule/DCMWalkingReferenceTrajectoryModule.hpp>
#include <PnC/PlannerSet/ContactSequenceGenerator/FootstepSequenceGenerator.hpp>
#include <PnC/DracoPnC/PredictionModule/DracoFootstep.hpp>

DCMPhaseWalkingTest::DCMPhaseWalkingTest(RobotSystem* robot) : Test(robot) {
    myUtils::pretty_constructor(1, "DCM Phase Walking Test");
    cfg_ = YAML::LoadFile(
            THIS_COM"Config/Draco/TEST/DCM_PHASE_WALKING_TEST.yaml");

    num_step_ = 0;
    phase_ = static_cast<int>(DCMPhaseWalkingTestPhase::DCMPhaseWalkingTestPhase_initial_jpos);

    // Trajectory Module
    std::vector<int> contact_index_to_side = {DRACO_RIGHT_FOOTSTEP, DRACO_RIGHT_FOOTSTEP,
                                              DRACO_LEFT_FOOTSTEP, DRACO_LEFT_FOOTSTEP};
    reference_trajectory_module_ = new DCMWalkingReferenceTrajectoryModule(contact_index_to_side);

    // Controllers
    jpos_target_ctrl_ = new IVDJPosTargetCtrl(robot);
    stand_up_ctrl_ = new StandUpCtrl(robot);

    foot_sequence_generator_ = new FootstepSequenceGenerator();
    ds_ctrl_ = new DoubleSupportCtrl(robot, reference_trajectory_module_,
            foot_sequence_generator_);

    right_swing_ctrl_ = new SingleSupportCtrl(robot, reference_trajectory_module_);
    left_swing_ctrl_ = new SingleSupportCtrl(robot,
            reference_trajectory_module_);

    right_swing_start_ctrl_ = new TransitionCtrl(robot,
            reference_trajectory_module_);
    right_swing_end_ctrl_ = new TransitionCtrl(robot,
            reference_trajectory_module_);
    left_swing_start_ctrl_ = new TransitionCtrl(robot,
            reference_trajectory_module_);
    left_swing_end_ctrl_ = new TransitionCtrl(robot,
            reference_trajectory_module_);

    state_list_.clear();
    state_list_.push_back(jpos_target_ctrl_);
    state_list_.push_back(stand_up_ctrl_);
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
    delete reference_trajectory_module_;
    delete foot_sequence_generator_;
}

void DCMPhaseWalkingTest::TestInitialization() {
    jpos_target_ctrl_->ctrlInitialization(
            cfg_["control_configuration"]["joint_position_ctrl"]);
    stand_up_ctrl_->ctrlInitialization(
            cfg_["control_configuration"]["stand_up_ctrl"]);
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

    if (phase == static_cast<int>(DCMPhaseWalkingTestPhase::DCMPhaseWalkingTestPhase_double_support_ctrl_1)) {
        ++num_step_;
        sp_->stance_foot = "lFoot";
        sp_->num_residual_steps -= 1;
    }

    if (phase == static_cast<int>(DCMPhaseWalkingTestPhase::DCMPhaseWalkingTestPhase_double_support_ctrl_2)) {
        ++num_step_;
        sp_->stance_foot = "rFoot";
        sp_->num_residual_steps -= 1;
    }
    sp_->num_step_copy = num_step_;

    if (next_phase == static_cast<int>(NUM_DCMPhaseWalkingTestPhase)) {
        return static_cast<int>(DCMPhaseWalkingTestPhase::DCMPhaseWalkingTestPhase_double_support_ctrl_1);
    } else {
        return next_phase;
    }
}

void DCMPhaseWalkingTest::_SettingParameter() {
    try {
        YAML::Node test_cfg = cfg_["test_configuration"];
        Eigen::VectorXd tmp_vec;
        double tmp_val;

        // Initial Joint Position
        myUtils::readParameter(test_cfg, "initial_jpos", tmp_vec);
        ((IVDJPosTargetCtrl*)jpos_target_ctrl_)->setTargetPosition(tmp_vec);

        // Joint Position Control Duration
        myUtils::readParameter(test_cfg, "jpos_initialization_time", tmp_val);
        ((IVDJPosTargetCtrl*)jpos_target_ctrl_)->setMovingTime(tmp_val);
        myUtils::readParameter(test_cfg, "jpos_ctrl_time", tmp_val);
        ((IVDJPosTargetCtrl*)jpos_target_ctrl_)->setTotalCtrlTime(tmp_val);

        // Target CoM Height
        myUtils::readParameter(test_cfg, "com_height", tmp_val);
        ((StandUpCtrl*)stand_up_ctrl_)->setCoMHeight(tmp_val);
        ((DoubleSupportCtrl*)ds_ctrl_)->setCoMHeight(tmp_val);
        ((SingleSupportCtrl*)right_swing_ctrl_)->setCoMHeight(tmp_val);
        ((SingleSupportCtrl*)left_swing_ctrl_)->setCoMHeight(tmp_val);
        ((TransitionCtrl*)right_swing_start_ctrl_)->setCoMHeight(tmp_val);
        ((TransitionCtrl*)right_swing_end_ctrl_)->setCoMHeight(tmp_val);
        ((TransitionCtrl*)left_swing_start_ctrl_)->setCoMHeight(tmp_val);
        ((TransitionCtrl*)left_swing_end_ctrl_)->setCoMHeight(tmp_val);
        ((DCMWalkingReferenceTrajectoryModule*)reference_trajectory_module_)->
            dcm_reference.setCoMHeight(tmp_val);
        sp_->omega = sqrt(9.81/tmp_val);

        // Footstep Sequence
        myUtils::readParameter(test_cfg, "footstep_length", tmp_val);
        foot_sequence_generator_->SetFootStepLength(tmp_val);

        myUtils::readParameter(test_cfg, "footstep_width", tmp_val);
        foot_sequence_generator_->SetRightFootStepWidth(tmp_val);
        foot_sequence_generator_->SetLeftFootStepWidth(tmp_val);

        myUtils::readParameter(test_cfg, "footstep_orientation", tmp_val);
        Eigen::Quaternion<double> quat_temp =
            Eigen::Quaternion<double>(cos(tmp_val/2.0), 0., 0., sin(tmp_val/2.0));
        tmp_vec = Eigen::VectorXd::Zero(4);
        tmp_vec << quat_temp.w(), quat_temp.x(), quat_temp.y(), quat_temp.z();
        foot_sequence_generator_->SetFootStepOrientation(tmp_vec);


        // Stand Up Control Duration
        myUtils::readParameter(test_cfg, "stand_up_ctrl_time", tmp_val);
        ((StandUpCtrl*)stand_up_ctrl_)->setTotalCtrlTime(tmp_val);

        // Double Support Control Duration
        double double_support_ctrl_time, initial_double_support_ctrl_time;
        myUtils::readParameter(test_cfg, "double_support_ctrl_time",
                double_support_ctrl_time);
        ((DoubleSupportCtrl*)ds_ctrl_)->setDoubleSupportDuration(
            double_support_ctrl_time);
        myUtils::readParameter(test_cfg, "initial_double_support_ctrl_time",
                initial_double_support_ctrl_time);
        ((DoubleSupportCtrl*)ds_ctrl_)->setInitialDoubleSupportDuration(
            initial_double_support_ctrl_time);

        // alpha_ds
        myUtils::readParameter(test_cfg, "alpha_ratio", tmp_val);
        ((DoubleSupportCtrl*)ds_ctrl_)->setAlphaRatio(tmp_val);
        ((DCMWalkingReferenceTrajectoryModule*)reference_trajectory_module_)->
            dcm_reference.alpha_ds = tmp_val;

        // Transition Control Duration
        double transition_ctrl_time;
        myUtils::readParameter(test_cfg, "transition_ctrl_time",
                transition_ctrl_time);
        ((TransitionCtrl*)right_swing_start_ctrl_)->setTotalCtrlTime(
            transition_ctrl_time);
        ((TransitionCtrl*)right_swing_end_ctrl_)->setTotalCtrlTime(
            transition_ctrl_time);
        ((TransitionCtrl*)left_swing_start_ctrl_)->setTotalCtrlTime(
            transition_ctrl_time);
        ((TransitionCtrl*)left_swing_end_ctrl_)->setTotalCtrlTime(
            transition_ctrl_time);

        // Single Support Control Duration
        double single_support_ctrl_time;
        myUtils::readParameter(test_cfg, "single_support_ctrl_time", 
                single_support_ctrl_time);
        ((SingleSupportCtrl*)right_swing_ctrl_)->setTotalCtrlTime(
            single_support_ctrl_time);
        ((SingleSupportCtrl*)left_swing_ctrl_)->setTotalCtrlTime(
            single_support_ctrl_time);

        // Swing Height
        myUtils::readParameter(test_cfg, "swing_foot_height", tmp_val);
        ((SingleSupportCtrl*)right_swing_ctrl_)->setSwingFootHeight(tmp_val);
        ((SingleSupportCtrl*)left_swing_ctrl_)->setSwingFootHeight(tmp_val);

        // Temporal Parameters
        // Note that t_transfer will be set in DoubleSupportCtrl.cpp since it is
        // varying.
        ((DCMWalkingReferenceTrajectoryModule*)reference_trajectory_module_)->
            dcm_reference.t_ds = double_support_ctrl_time;
        ((DCMWalkingReferenceTrajectoryModule*)reference_trajectory_module_)->
            dcm_reference.t_ss = single_support_ctrl_time +
            2*transition_ctrl_time;

    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
        exit(0);
    }
}
