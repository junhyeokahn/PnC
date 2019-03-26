#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/DracoPnC/TestSet/TestSet.hpp>
#include <PnC/PlannerSet/PIPM_FootPlacementPlanner/Reversal_LIPM_Planner.hpp>
#include <Utils/IO/DataManager.hpp>

TurningTest::TurningTest(RobotSystem* robot) : Test(robot) {
    myUtils::pretty_constructor(1, "Turning Test");
    // cfg_ = YAML::LoadFile(THIS_COM"Config/Draco/TEST/TURNING_EXP_TEST.yaml");
    cfg_ = YAML::LoadFile(THIS_COM "Config/Draco/TEST/TURNING_SIM_TEST.yaml");

    num_step_ = 0;
    sp_ = DracoStateProvider::getStateProvider(robot_);
    sp_->stance_foot = "lFoot";
    sp_->global_pos_local[1] = 0.15;
    reversal_planner_ = new Reversal_LIPM_Planner();
    phase_ = TnPhase::initiation;

    state_list_.clear();

    jpos_ctrl_ = new JPosTargetCtrl(robot);
    body_up_ctrl_ = new DoubleContactTransCtrl(robot);
    body_fix_ctrl_ = new BodyCtrl(robot);

    // Swing Controller Selection
    right_swing_ctrl_ =
        new BodyFootTurningCtrl(robot_, "rFoot", reversal_planner_);
    left_swing_ctrl_ =
        new BodyFootTurningCtrl(robot_, "lFoot", reversal_planner_);

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

    DataManager::GetDataManager()->RegisterData(
        &(((SwingPlanningCtrl*)right_swing_ctrl_)->curr_foot_quat_des_),
        QUATERNION, "rfoot_quat_des", 4);

    DataManager::GetDataManager()->RegisterData(
        &(((SwingPlanningCtrl*)left_swing_ctrl_)->curr_foot_quat_des_),
        QUATERNION, "lfoot_quat_des", 4);

    DataManager::GetDataManager()->RegisterData(
        &(((SwingPlanningCtrl*)right_swing_ctrl_)->curr_foot_so3_des_), VECT3,
        "rfoot_so3_des", 3);

    DataManager::GetDataManager()->RegisterData(
        &(((SwingPlanningCtrl*)left_swing_ctrl_)->curr_foot_so3_des_), VECT3,
        "lfoot_so3_des", 3);
}

TurningTest::~TurningTest() {
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

void TurningTest::TestInitialization() {
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
        cfg_["control_configuration"]["right_body_foot_turning_ctrl"]);
    left_swing_ctrl_->ctrlInitialization(
        cfg_["control_configuration"]["left_body_foot_turning_ctrl"]);
}

int TurningTest::_NextPhase(const int& phase) {
    int next_phase = phase + 1;
    myUtils::color_print(myColor::BoldGreen,
                         "[Phase " + std::to_string(next_phase) + "]");
    Eigen::Vector3d next_local_frame_location;

    if (phase == TnPhase::double_contact_1) {
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
    if (phase == TnPhase::double_contact_2) {
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
    // if (((phase == TnPhase::double_contact_1) ||
    //(phase == TnPhase::double_contact_2)) &&
    //(num_step_ > 1)) {
    // sp_->global_pos_local[0] =
    // sp_->first_LED_x + (next_local_frame_location[0] - sp_->q[0]);
    // sp_->global_pos_local[1] =
    // sp_->first_LED_y + (next_local_frame_location[1] - sp_->q[1]);
    //}

    // Update Turning Target
    if (sp_->curr_time > walking_start_time_) {
        if (sp_->phase_copy == 2 || sp_->phase_copy == 6) {
            sp_->des_quat = delta_quat_ * sp_->des_quat;
        }
    }

    if (next_phase == TnPhase::NUM_WALKING_PHASE) {
        return TnPhase::double_contact_1;
    } else {
        return next_phase;
    }
}

void TurningTest::_SettingParameter() {
    try {
        double tmp;
        bool b_tmp;
        Eigen::VectorXd tmp_vec(10);
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

        // walking and turning parameters
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
        //!! TEST !!
        // delta_quat_ = Eigen::Quaternion<double>(1, 0, 0, 0);
        walking_velocity_ << 0, 0;
        //!! TEST !!
        // myUtils::pretty_print((Eigen::VectorXd)walking_velocity_, std::cout,
        //"walking_velocity");
        // myUtils::pretty_print(delta_quat_, std::cout, "delta_quat");

    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}

void TurningTest::AdditionalUpdate_() {
    // Update Walking Target
    if (sp_->curr_time > walking_start_time_) {
        for (int i = 0; i < 2; ++i) {
            sp_->walking_velocity[i] = walking_velocity_[i];
            sp_->des_location[i] += walking_velocity_[i] * DracoAux::ServoRate;
        }
    }
}
