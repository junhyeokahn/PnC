#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <ReinforcementLearning/RLInterface/RLInterface.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/DracoPnC/TestSet/TestSet.hpp>
#include <PnC/PlannerSet/PIPM_FootPlacementPlanner/Reversal_LIPM_Planner.hpp>
#include <Utils/IO/DataManager.hpp>

RLWalkingTest::RLWalkingTest(RobotSystem* robot, int mpi_idx, int env_idx) : Test(robot) {
    myUtils::pretty_constructor(1, "Walking Test");
    cfg_ = YAML::LoadFile(THIS_COM "Config/Draco/TEST/RL_WALKING_TEST.yaml");
    mpi_idx_ = mpi_idx;
    env_idx_ = env_idx;
    b_learning_ = true;

    num_step_ = 0;
    sp_ = DracoStateProvider::getStateProvider(robot_);
    sp_->stance_foot = "lFoot";
    sp_->global_pos_local[1] = 0.15;
    reversal_planner_ = new Reversal_LIPM_Planner();
    phase_ = WkPhase::initiation;

    state_list_.clear();

    jpos_ctrl_ = new JPosTargetCtrl(robot);
    body_up_ctrl_ = new DoubleContactTransCtrl(robot);
    body_fix_ctrl_ = new BodyCtrl(robot);
    // Swing Controller Selection
    right_swing_ctrl_ =
        new BodyFootLearningCtrl(robot_, "rFoot", reversal_planner_);
    left_swing_ctrl_ =
        new BodyFootLearningCtrl(robot_, "lFoot", reversal_planner_);

    // Right
    right_swing_start_trans_ctrl_ =
        new SingleContactTransCtrl(robot, "rFoot", false);
    right_swing_end_trans_ctrl_ =
        new SingleContactTransCtrl(robot, "rFoot", true);
    // Left
    left_swing_start_trans_ctrl_ =
        new SingleContactTransCtrl(robot, "lFoot", false);
    left_swing_end_trans_ctrl_ =
        new SingleContactTransCtrl(robot, "lFoot", true);

    _SettingParameter();

    state_list_.push_back(jpos_ctrl_);
    state_list_.push_back(body_up_ctrl_);
    state_list_.push_back(body_fix_ctrl_);
    state_list_.push_back(right_swing_start_trans_ctrl_);
    state_list_.push_back(right_swing_ctrl_);
    state_list_.push_back(right_swing_end_trans_ctrl_);
    state_list_.push_back(body_fix_ctrl_);
    state_list_.push_back(left_swing_start_trans_ctrl_);
    state_list_.push_back(left_swing_ctrl_);
    state_list_.push_back(left_swing_end_trans_ctrl_);

    DataManager::GetDataManager()->RegisterData(
        &(((SwingPlanningCtrl*)right_swing_ctrl_)->curr_foot_pos_des_), VECT3,
        "rfoot_pos_des", 3);
    DataManager::GetDataManager()->RegisterData(
        &(((SwingPlanningCtrl*)left_swing_ctrl_)->curr_foot_pos_des_), VECT3,
        "lfoot_pos_des", 3);

    DataManager::GetDataManager()->RegisterData(
        &(((SwingPlanningCtrl*)right_swing_ctrl_)->curr_foot_vel_des_), VECT3,
        "rfoot_vel_des", 3);
    DataManager::GetDataManager()->RegisterData(
        &(((SwingPlanningCtrl*)left_swing_ctrl_)->curr_foot_vel_des_), VECT3,
        "lfoot_vel_des", 3);

    DataManager::GetDataManager()->RegisterData(
        &(((SwingPlanningCtrl*)right_swing_ctrl_)->curr_foot_acc_des_), VECT3,
        "rfoot_acc_des", 3);
    DataManager::GetDataManager()->RegisterData(
        &(((SwingPlanningCtrl*)left_swing_ctrl_)->curr_foot_acc_des_), VECT3,
        "lfoot_acc_des", 3);
}

RLWalkingTest::RLWalkingTest(RobotSystem* robot) : Test(robot) {
/*
 *    myUtils::pretty_constructor(1, "Walking Test");
 *    cfg_ = YAML::LoadFile(THIS_COM "Config/Draco/TEST/RL_WALKING_TEST.yaml");
 *    mpi_idx_ = 0
 *    env_idx_ = 0
 *    b_learning_ = false;
 *
 *    num_step_ = 0;
 *    sp_ = DracoStateProvider::getStateProvider(robot_);
 *    sp_->stance_foot = "lFoot";
 *    sp_->global_pos_local[1] = 0.15;
 *    reversal_planner_ = new Reversal_LIPM_Planner();
 *    phase_ = WkPhase::initiation;
 *
 *    state_list_.clear();
 *
 *    jpos_ctrl_ = new JPosTargetCtrl(robot);
 *    body_up_ctrl_ = new DoubleContactTransCtrl(robot);
 *    body_fix_ctrl_ = new BodyCtrl(robot);
 *    // Swing Controller Selection
 *    right_swing_ctrl_ =
 *        new BodyFootPlanningCtrl(robot_, "rFoot", reversal_planner_);
 *    left_swing_ctrl_ =
 *        new BodyFootPlanningCtrl(robot_, "lFoot", reversal_planner_);
 *
 *    // Right
 *    right_swing_start_trans_ctrl_ =
 *        new SingleContactTransCtrl(robot, "rFoot", false);
 *    right_swing_end_trans_ctrl_ =
 *        new SingleContactTransCtrl(robot, "rFoot", true);
 *    // Left
 *    left_swing_start_trans_ctrl_ =
 *        new SingleContactTransCtrl(robot, "lFoot", false);
 *    left_swing_end_trans_ctrl_ =
 *        new SingleContactTransCtrl(robot, "lFoot", true);
 *
 *    _SettingParameter();
 *
 *    state_list_.push_back(jpos_ctrl_);
 *    state_list_.push_back(body_up_ctrl_);
 *    state_list_.push_back(body_fix_ctrl_);
 *    state_list_.push_back(right_swing_start_trans_ctrl_);
 *    state_list_.push_back(right_swing_ctrl_);
 *    state_list_.push_back(right_swing_end_trans_ctrl_);
 *    state_list_.push_back(body_fix_ctrl_);
 *    state_list_.push_back(left_swing_start_trans_ctrl_);
 *    state_list_.push_back(left_swing_ctrl_);
 *    state_list_.push_back(left_swing_end_trans_ctrl_);
 *
 *    DataManager::GetDataManager()->RegisterData(
 *        &(((SwingPlanningCtrl*)right_swing_ctrl_)->curr_foot_pos_des_), VECT3,
 *        "rfoot_pos_des", 3);
 *    DataManager::GetDataManager()->RegisterData(
 *        &(((SwingPlanningCtrl*)left_swing_ctrl_)->curr_foot_pos_des_), VECT3,
 *        "lfoot_pos_des", 3);
 *
 *    DataManager::GetDataManager()->RegisterData(
 *        &(((SwingPlanningCtrl*)right_swing_ctrl_)->curr_foot_vel_des_), VECT3,
 *        "rfoot_vel_des", 3);
 *    DataManager::GetDataManager()->RegisterData(
 *        &(((SwingPlanningCtrl*)left_swing_ctrl_)->curr_foot_vel_des_), VECT3,
 *        "lfoot_vel_des", 3);
 *
 *    DataManager::GetDataManager()->RegisterData(
 *        &(((SwingPlanningCtrl*)right_swing_ctrl_)->curr_foot_acc_des_), VECT3,
 *        "rfoot_acc_des", 3);
 *    DataManager::GetDataManager()->RegisterData(
 *        &(((SwingPlanningCtrl*)left_swing_ctrl_)->curr_foot_acc_des_), VECT3,
 *        "lfoot_acc_des", 3);
 */
    printf("Not Implemented yet\n");
}

RLWalkingTest::~RLWalkingTest() {
    delete jpos_ctrl_;
    delete body_up_ctrl_;
    delete right_swing_start_trans_ctrl_;
    delete right_swing_ctrl_;
    delete right_swing_end_trans_ctrl_;
    delete body_fix_ctrl_;
    delete left_swing_start_trans_ctrl_;
    delete left_swing_ctrl_;
    delete left_swing_end_trans_ctrl_;

    delete reversal_planner_;
}

void RLWalkingTest::TestInitialization() {
    reversal_planner_->PlannerInitialization(
        cfg_["planner_configuration"]["velocity_reversal_pln"]);

    jpos_ctrl_->ctrlInitialization(
        cfg_["control_configuration"]["joint_position_ctrl"]);
    body_up_ctrl_->ctrlInitialization(
        cfg_["control_configuration"]["double_contact_trans_ctrl"]);
    body_fix_ctrl_->ctrlInitialization(
        cfg_["control_configuration"]["body_ctrl"]);

    right_swing_start_trans_ctrl_->ctrlInitialization(
        cfg_["control_configuration"]["single_contact_trans_ctrl"]);
    right_swing_end_trans_ctrl_->ctrlInitialization(
        cfg_["control_configuration"]["single_contact_trans_ctrl"]);
    left_swing_start_trans_ctrl_->ctrlInitialization(
        cfg_["control_configuration"]["single_contact_trans_ctrl"]);
    left_swing_end_trans_ctrl_->ctrlInitialization(
        cfg_["control_configuration"]["single_contact_trans_ctrl"]);

    right_swing_ctrl_->ctrlInitialization(
        cfg_["control_configuration"]["right_body_foot_planning_ctrl"]);
    left_swing_ctrl_->ctrlInitialization(
        cfg_["control_configuration"]["left_body_foot_planning_ctrl"]);
}

int RLWalkingTest::_NextPhase(const int& phase) {
    for (int i = 0; i < 2; ++i) {
        if (sp_->b_observe_keyframe_vel[i] && sp_->curr_time > sp_->contact_time + t_prime_[i]) {
            sp_->b_observe_keyframe_vel[i] = false;
            sp_->keyframe_vel[i] = sp_->qdot[i];
        }
    }
    if ( !(sp_->b_observe_keyframe_vel[0] || sp_->b_observe_keyframe_vel[1]) &&
            RLInterface::GetRLInterface()->GetRLData()->b_data_filled) {
        for (int i = 0; i < 2; ++i) {
            RLInterface::GetRLInterface()->GetRLData()->reward -=
                keyframe_vel_penalty_[i] * (sp_->target_keyframe_vel[i] - sp_->keyframe_vel[i]) * (sp_->target_keyframe_vel[i] - sp_->keyframe_vel[i]);
        }
        RLInterface::GetRLInterface()->SendData();
    }

    int next_phase = phase + 1;
    myUtils::color_print(myColor::BoldGreen,
                         "[Phase " + std::to_string(next_phase) + "]");
    Eigen::Vector3d next_local_frame_location;

    if (phase == WkPhase::double_contact_1) {
        ++num_step_;
        printf("%i th step : ", num_step_);
        printf("Right Leg Swing\n");
        sp_->stance_foot = "lFoot";

        // Global Frame Update
        next_local_frame_location =
            robot_->getBodyNodeIsometry(DracoBodyNode::lFootCenter)
                .translation();
        sp_->global_pos_local += next_local_frame_location;
    }
    if (phase == WkPhase::double_contact_2) {
        ++num_step_;
        printf("%i th step : ", num_step_);
        printf("Left Leg Swing\n");

        sp_->stance_foot = "rFoot";

        // Global Frame Update
        next_local_frame_location =
            robot_->getBodyNodeIsometry(DracoBodyNode::rFootCenter)
                .translation();
        sp_->global_pos_local += next_local_frame_location;
    }
    sp_->num_step_copy = num_step_;

    // !! TEST !!
    // if (((phase == WkPhase::double_contact_1) ||
    //(phase == WkPhase::double_contact_2)) &&
    //(num_step_ > 1)) {
    // sp_->global_pos_local[0] =
    // sp_->first_LED_x + (next_local_frame_location[0] - sp_->q[0]);
    // sp_->global_pos_local[1] =
    // sp_->first_LED_y + (next_local_frame_location[1] - sp_->q[1]);
    //}

    if (next_phase == WkPhase::NUM_WALKING_PHASE) {
        return WkPhase::double_contact_1;
    } else {
        return next_phase;
    }
}

void RLWalkingTest::_SettingParameter() {
    try {
        double tmp;
        bool b_tmp;
        Eigen::VectorXd tmp_vec;
        std::string tmp_str;

        YAML::Node test_cfg = cfg_["test_configuration"];

        myUtils::readParameter(test_cfg, "start_phase", phase_);

        myUtils::readParameter(test_cfg, "initial_jpos", tmp_vec);
        ((JPosTargetCtrl*)jpos_ctrl_)->setTargetPosition(tmp_vec);

        myUtils::readParameter(test_cfg, "body_height", tmp);
        ((DoubleContactTransCtrl*)body_up_ctrl_)->setStanceHeight(tmp);
        ((BodyCtrl*)body_fix_ctrl_)->setStanceHeight(tmp);
        ((SingleContactTransCtrl*)right_swing_start_trans_ctrl_)
            ->setStanceHeight(tmp);
        ((SingleContactTransCtrl*)right_swing_end_trans_ctrl_)
            ->setStanceHeight(tmp);
        ((SingleContactTransCtrl*)left_swing_start_trans_ctrl_)
            ->setStanceHeight(tmp);
        ((SingleContactTransCtrl*)left_swing_end_trans_ctrl_)
            ->setStanceHeight(tmp);
        ((SwingPlanningCtrl*)right_swing_ctrl_)->setStanceHeight(tmp);
        ((SwingPlanningCtrl*)left_swing_ctrl_)->setStanceHeight(tmp);

        myUtils::readParameter(test_cfg, "com_height_for_omega", tmp);
        ((Reversal_LIPM_Planner*)reversal_planner_)->setOmega(tmp);

        myUtils::readParameter(test_cfg, "jpos_initialization_time", tmp);
        ((JPosTargetCtrl*)jpos_ctrl_)->setMovingTime(tmp);

        myUtils::readParameter(test_cfg, "com_lifting_time", tmp);
        ((DoubleContactTransCtrl*)body_up_ctrl_)->setStanceTime(tmp);

        myUtils::readParameter(test_cfg, "stance_time", tmp);
        ((SwingPlanningCtrl*)right_swing_ctrl_)->notifyStanceTime(tmp);
        ((SwingPlanningCtrl*)left_swing_ctrl_)->notifyStanceTime(tmp);

        myUtils::readParameter(test_cfg, "swing_time", tmp);
        ((SwingPlanningCtrl*)right_swing_ctrl_)->setSwingTime(tmp);
        ((SwingPlanningCtrl*)left_swing_ctrl_)->setSwingTime(tmp);

        myUtils::readParameter(test_cfg, "st_transition_time", tmp);
        ((SingleContactTransCtrl*)right_swing_start_trans_ctrl_)
            ->setTransitionTime(tmp);
        ((SingleContactTransCtrl*)right_swing_end_trans_ctrl_)
            ->setTransitionTime(tmp);
        ((SingleContactTransCtrl*)left_swing_start_trans_ctrl_)
            ->setTransitionTime(tmp);
        ((SingleContactTransCtrl*)left_swing_end_trans_ctrl_)
            ->setTransitionTime(tmp);
        ((SwingPlanningCtrl*)right_swing_ctrl_)->notifyTransitionTime(tmp);
        ((SwingPlanningCtrl*)left_swing_ctrl_)->notifyTransitionTime(tmp);

        myUtils::readParameter(test_cfg, "replanning", b_tmp);
        ((SwingPlanningCtrl*)right_swing_ctrl_)->setReplanning(b_tmp);
        ((SwingPlanningCtrl*)left_swing_ctrl_)->setReplanning(b_tmp);

        myUtils::readParameter(test_cfg, "double_stance_mix_ratio", tmp);
        ((SwingPlanningCtrl*)right_swing_ctrl_)->setDoubleStanceRatio(tmp);
        ((SwingPlanningCtrl*)left_swing_ctrl_)->setDoubleStanceRatio(tmp);

        myUtils::readParameter(test_cfg, "transition_phase_mix_ratio", tmp);
        ((SwingPlanningCtrl*)right_swing_ctrl_)->setTransitionPhaseRatio(tmp);
        ((SwingPlanningCtrl*)left_swing_ctrl_)->setTransitionPhaseRatio(tmp);

        myUtils::readParameter(test_cfg, "contact_switch_check", b_tmp);
        ((SwingPlanningCtrl*)right_swing_ctrl_)->setContactSwitchCheck(b_tmp);
        ((SwingPlanningCtrl*)left_swing_ctrl_)->setContactSwitchCheck(b_tmp);

        if (b_learning_) {
            RLInterface* rl_interface = RLInterface::GetRLInterface();
            rl_interface->Initialize(test_cfg["protocol"], mpi_idx_, env_idx_);
            ((BodyFootLearningCtrl*)right_swing_ctrl_)->setPolicy(rl_interface->GetPolicy());
            ((BodyFootLearningCtrl*)right_swing_ctrl_)->setValueFn(rl_interface->GetValueFn());
            ((BodyFootLearningCtrl*)left_swing_ctrl_)->setPolicy(rl_interface->GetPolicy());
            ((BodyFootLearningCtrl*)left_swing_ctrl_)->setValueFn(rl_interface->GetValueFn());
            myUtils::readParameter(test_cfg, "action_lower_bound", tmp_vec);
            ((BodyFootLearningCtrl*)left_swing_ctrl_)->setActionLowerBound(tmp_vec);
            ((BodyFootLearningCtrl*)right_swing_ctrl_)->setActionLowerBound(tmp_vec);
            myUtils::readParameter(test_cfg, "action_upper_bound", tmp_vec);
            ((BodyFootLearningCtrl*)left_swing_ctrl_)->setActionUpperBound(tmp_vec);
            ((BodyFootLearningCtrl*)right_swing_ctrl_)->setActionUpperBound(tmp_vec);
            myUtils::readParameter(test_cfg, "action_sclae", tmp_vec);
            ((BodyFootLearningCtrl*)left_swing_ctrl_)->setActionScale(tmp_vec);
            ((BodyFootLearningCtrl*)right_swing_ctrl_)->setActionScale(tmp_vec);
            myUtils::readParameter(test_cfg, "terminate_obs_lower_bound", tmp_vec);
            ((BodyFootLearningCtrl*)left_swing_ctrl_)->setTerminateObsLowerBound(tmp_vec);
            ((BodyFootLearningCtrl*)right_swing_ctrl_)->setTerminateObsLowerBound(tmp_vec);
            myUtils::readParameter(test_cfg, "terminate_obs_upper_bound", tmp_vec);
            ((BodyFootLearningCtrl*)left_swing_ctrl_)->setTerminateObsUpperBound(tmp_vec);
            ((BodyFootLearningCtrl*)right_swing_ctrl_)->setTerminateObsUpperBound(tmp_vec);
        }
        myUtils::readParameter(test_cfg, "keyframe_vel_penalty", keyframe_vel_penalty_);
        myUtils::readParameter(
                cfg_["planner_configuration"]["velocity_reversal_pln"],
                "t_prime", t_prime_);

    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}
