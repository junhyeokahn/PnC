#include <PnC/AtlasPnC/AtlasDefinition.hpp>
#include <PnC/AtlasPnC/AtlasStateProvider.hpp>
#include <PnC/AtlasPnC/ContactSet/ContactSet.hpp>
#include <PnC/AtlasPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/AtlasPnC/TaskSet/TaskSet.hpp>
#include <PnC/AtlasPnC/TestSet/RLWalkingTest.hpp>
#include <PnC/PlannerSet/LIPMPlanner/TVRPlanner.hpp>
#include <ReinforcementLearning/RLInterface/RLInterface.hpp>
#include <Utils/IO/DataManager.hpp>

RLWalkingTest::RLWalkingTest(RobotSystem* robot) : Test(robot) {
    myUtils::pretty_constructor(1, "RL Walking Test");
    cfg_ = YAML::LoadFile(THIS_COM "Config/Atlas/TEST/RL_WALKING_TEST.yaml");

    num_step_ = 0;
    mpi_idx_ = 0;
    env_idx_ = 0;
    b_learning_ = false;
    sp_ = AtlasStateProvider::getStateProvider(robot_);
    sp_->stance_foot = AtlasBodyNode::l_sole;
    sp_->global_pos_local[1] = 0.11;
    reversal_planner_ = new TVRPlanner();
    phase_ = RLWkPhase::double_contact_1;

    state_list_.clear();

    body_fix_ctrl_ = new BodyCtrl(robot);
    // Right
    right_swing_start_trans_ctrl_ =
        new SingleContactTransCtrl(robot, AtlasBodyNode::r_sole, false);
    right_swing_end_trans_ctrl_ =
        new SingleContactTransCtrl(robot, AtlasBodyNode::r_sole, true);
    // Left
    left_swing_start_trans_ctrl_ =
        new SingleContactTransCtrl(robot, AtlasBodyNode::l_sole, false);
    left_swing_end_trans_ctrl_ =
        new SingleContactTransCtrl(robot, AtlasBodyNode::l_sole, true);
    // Swing Controller Selection
    right_swing_ctrl_ = new BodyFootPolicyCtrl(robot_, AtlasBodyNode::r_sole,
                                               reversal_planner_);
    left_swing_ctrl_ = new BodyFootPolicyCtrl(robot_, AtlasBodyNode::l_sole,
                                              reversal_planner_);
    _SettingParameter();

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

RLWalkingTest::RLWalkingTest(RobotSystem* robot, int mpi_idx, int env_idx)
    : Test(robot) {
    myUtils::pretty_constructor(1, "RL Walking Test");
    cfg_ = YAML::LoadFile(THIS_COM "Config/Atlas/TEST/RL_WALKING_TEST.yaml");

    num_step_ = 0;
    mpi_idx_ = mpi_idx;
    env_idx_ = env_idx;
    b_learning_ = true;
    sp_ = AtlasStateProvider::getStateProvider(robot_);
    sp_->stance_foot = AtlasBodyNode::l_sole;
    sp_->global_pos_local[1] = 0.11;
    reversal_planner_ = new TVRPlanner();
    phase_ = RLWkPhase::double_contact_1;

    state_list_.clear();

    body_fix_ctrl_ = new BodyCtrl(robot);
    // Right
    right_swing_start_trans_ctrl_ =
        new SingleContactTransCtrl(robot, AtlasBodyNode::r_sole, false);
    right_swing_end_trans_ctrl_ =
        new SingleContactTransCtrl(robot, AtlasBodyNode::r_sole, true);
    // Left
    left_swing_start_trans_ctrl_ =
        new SingleContactTransCtrl(robot, AtlasBodyNode::l_sole, false);
    left_swing_end_trans_ctrl_ =
        new SingleContactTransCtrl(robot, AtlasBodyNode::l_sole, true);
    // Swing Controller Selection
    right_swing_ctrl_ = new BodyFootLearningCtrl(robot_, AtlasBodyNode::r_sole,
                                                 reversal_planner_);
    left_swing_ctrl_ = new BodyFootLearningCtrl(robot_, AtlasBodyNode::l_sole,
                                                reversal_planner_);
    _SettingParameter();

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

RLWalkingTest::~RLWalkingTest() {
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

    if (b_learning_) {
        right_swing_ctrl_->ctrlInitialization(
            cfg_["control_configuration"]["right_body_foot_learning_ctrl"]);
        left_swing_ctrl_->ctrlInitialization(
            cfg_["control_configuration"]["left_body_foot_learning_ctrl"]);
    } else {
        right_swing_ctrl_->ctrlInitialization(
            cfg_["control_configuration"]["right_body_foot_policy_ctrl"]);
        left_swing_ctrl_->ctrlInitialization(
            cfg_["control_configuration"]["left_body_foot_policy_ctrl"]);
    }
}

int RLWalkingTest::_NextPhase(const int& phase) {
    int next_phase = phase + 1;
    myUtils::color_print(myColor::BoldGreen,
                         "[Phase " + std::to_string(next_phase) + "]");
    Eigen::Vector3d next_local_frame_location;

    if (phase == RLWkPhase::double_contact_1) {
        ++num_step_;
        printf("%i th step : ", num_step_);
        printf("Right Leg Swing\n");
        sp_->stance_foot = AtlasBodyNode::l_sole;

        // Global Frame Update
        next_local_frame_location =
            robot_->getBodyNodeIsometry(AtlasBodyNode::l_sole).translation();
        sp_->global_pos_local += next_local_frame_location;
    }
    if (phase == RLWkPhase::double_contact_2) {
        ++num_step_;
        printf("%i th step : ", num_step_);
        printf("Left Leg Swing\n");

        sp_->stance_foot = AtlasBodyNode::r_sole;

        // Global Frame Update
        next_local_frame_location =
            robot_->getBodyNodeIsometry(AtlasBodyNode::r_sole).translation();
        sp_->global_pos_local += next_local_frame_location;
    }
    sp_->num_step_copy = num_step_;

    // Update Turning Target
    if (sp_->curr_time > walking_start_time_) {
        if (sp_->phase_copy == 2 || sp_->phase_copy == 6) {
            sp_->des_quat = delta_quat_ * sp_->des_quat;
        }
    }

    if (next_phase == RLWkPhase::NUM_WALKING_PHASE) {
        return RLWkPhase::double_contact_1;
    } else {
        return next_phase;
    }
}

void RLWalkingTest::_SettingParameter() {
    try {
        double tmp;
        bool b_tmp;
        Eigen::VectorXd tmp_vec(10);
        std::string tmp_str;

        YAML::Node test_cfg = cfg_["test_configuration"];

        // =====================================================================
        // body height
        // =====================================================================
        myUtils::readParameter(test_cfg, "body_height", tmp);

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
        ((TVRPlanner*)reversal_planner_)->setOmega(tmp);

        // =====================================================================
        // stance time
        // =====================================================================
        myUtils::readParameter(test_cfg, "stance_time", tmp);
        ((BodyCtrl*)body_fix_ctrl_)->setStanceTime(tmp);
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

        // =====================================================================
        // Locomotion Behavior
        // =====================================================================
        myUtils::readParameter(test_cfg, "walking_start_time",
                               walking_start_time_);
        Eigen::VectorXd walking_velocity_lb, walking_velocity_ub;
        walking_velocity_.setZero();
        myUtils::readParameter(test_cfg, "walking_velocity_lb",
                               walking_velocity_lb);
        myUtils::readParameter(test_cfg, "walking_velocity_ub",
                               walking_velocity_ub);
        double turning_rate_lb, turning_rate_ub;
        myUtils::readParameter(test_cfg, "turning_rate_lb", turning_rate_lb);
        myUtils::readParameter(test_cfg, "turning_rate_ub", turning_rate_ub);
        std::random_device rd;
        std::mt19937 gen(rd());
        for (int i = 0; i < 2; ++i) {
            std::uniform_real_distribution<> dis(walking_velocity_lb[i],
                                                 walking_velocity_ub[i]);
            walking_velocity_[i] = dis(gen);
        }
        std::uniform_real_distribution<> dis(turning_rate_lb, turning_rate_ub);
        turning_rate_ = dis(gen);
        Eigen::Vector3d euler_zyx(turning_rate_, 0, 0);
        delta_quat_ =
            Eigen::Quaternion<double>(dart::math::eulerZYXToMatrix(euler_zyx));

        // myUtils::pretty_print((Eigen::VectorXd)walking_velocity_, std::cout,
        //"walking_velocity");
        // myUtils::pretty_print(delta_quat_, std::cout, "delta_quat");

        // =====================================================================
        // Environment Parameters
        // =====================================================================
        if (b_learning_) {
            RLInterface* rl_interface = RLInterface::GetRLInterface();
            rl_interface->Initialize(test_cfg["protocol"], mpi_idx_, env_idx_);
            ((BodyFootLearningCtrl*)right_swing_ctrl_)
                ->setPolicy(rl_interface->GetPolicy());
            ((BodyFootLearningCtrl*)right_swing_ctrl_)
                ->setValueFn(rl_interface->GetValueFn());
            ((BodyFootLearningCtrl*)left_swing_ctrl_)
                ->setPolicy(rl_interface->GetPolicy());
            ((BodyFootLearningCtrl*)left_swing_ctrl_)
                ->setValueFn(rl_interface->GetValueFn());

            myUtils::readParameter(test_cfg, "action_lower_bound", tmp_vec);
            ((BodyFootLearningCtrl*)left_swing_ctrl_)
                ->setActionLowerBound(tmp_vec);
            ((BodyFootLearningCtrl*)right_swing_ctrl_)
                ->setActionLowerBound(tmp_vec);

            myUtils::readParameter(test_cfg, "action_upper_bound", tmp_vec);
            ((BodyFootLearningCtrl*)left_swing_ctrl_)
                ->setActionUpperBound(tmp_vec);
            ((BodyFootLearningCtrl*)right_swing_ctrl_)
                ->setActionUpperBound(tmp_vec);

            myUtils::readParameter(test_cfg, "action_scale", tmp_vec);
            ((BodyFootLearningCtrl*)left_swing_ctrl_)->setActionScale(tmp_vec);
            ((BodyFootLearningCtrl*)right_swing_ctrl_)->setActionScale(tmp_vec);

            myUtils::readParameter(test_cfg, "terminate_obs_lower_bound",
                                   tmp_vec);
            ((BodyFootLearningCtrl*)left_swing_ctrl_)
                ->setTerminateObsLowerBound(tmp_vec);
            ((BodyFootLearningCtrl*)right_swing_ctrl_)
                ->setTerminateObsLowerBound(tmp_vec);
            myUtils::readParameter(test_cfg, "terminate_obs_upper_bound",
                                   tmp_vec);
            ((BodyFootLearningCtrl*)left_swing_ctrl_)
                ->setTerminateObsUpperBound(tmp_vec);
            ((BodyFootLearningCtrl*)right_swing_ctrl_)
                ->setTerminateObsUpperBound(tmp_vec);
        } else {
            myUtils::readParameter(test_cfg, "action_lower_bound", tmp_vec);
            ((BodyFootPolicyCtrl*)right_swing_ctrl_)
                ->setActionLowerBound(tmp_vec);
            ((BodyFootPolicyCtrl*)left_swing_ctrl_)
                ->setActionLowerBound(tmp_vec);

            myUtils::readParameter(test_cfg, "action_upper_bound", tmp_vec);
            ((BodyFootPolicyCtrl*)right_swing_ctrl_)
                ->setActionUpperBound(tmp_vec);
            ((BodyFootPolicyCtrl*)left_swing_ctrl_)
                ->setActionUpperBound(tmp_vec);

            myUtils::readParameter(test_cfg, "action_scale", tmp_vec);
            ((BodyFootPolicyCtrl*)right_swing_ctrl_)->setActionScale(tmp_vec);
            ((BodyFootPolicyCtrl*)left_swing_ctrl_)->setActionScale(tmp_vec);

            myUtils::readParameter(test_cfg, "terminate_obs_lower_bound",
                                   tmp_vec);
            ((BodyFootPolicyCtrl*)left_swing_ctrl_)
                ->setTerminateObsLowerBound(tmp_vec);
            ((BodyFootPolicyCtrl*)right_swing_ctrl_)
                ->setTerminateObsLowerBound(tmp_vec);

            myUtils::readParameter(test_cfg, "terminate_obs_upper_bound",
                                   tmp_vec);
            ((BodyFootPolicyCtrl*)left_swing_ctrl_)
                ->setTerminateObsUpperBound(tmp_vec);
            ((BodyFootPolicyCtrl*)right_swing_ctrl_)
                ->setTerminateObsUpperBound(tmp_vec);
        }

    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}

void RLWalkingTest::AdditionalUpdate_() {
    // Update Walking Target
    if (sp_->curr_time > walking_start_time_) {
        for (int i = 0; i < 2; ++i) {
            sp_->walking_velocity[i] = walking_velocity_[i];
            sp_->des_location[i] += walking_velocity_[i] * AtlasAux::servo_rate;
        }
    }
}
