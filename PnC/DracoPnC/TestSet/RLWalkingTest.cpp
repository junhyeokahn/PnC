#include <PnC/DracoPnC/TestSet/TestSet.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/PlannerSet/PIPM_FootPlacementPlanner/Reversal_LIPM_Planner.hpp>
#include <Utils/DataManager.hpp>

RLWalkingTest::RLWalkingTest(RobotSystem* robot) : Test(robot) {
    myUtils::pretty_constructor(1, "RL Walking Test");
    cfg_ = YAML::LoadFile(THIS_COM"Config/Draco/TEST/RL_WALKING_TEST.yaml");

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
        new BodyFootPlanningCtrl(robot_, "rFoot", reversal_planner_);
    left_swing_ctrl_ =
        new BodyFootPlanningCtrl(robot_, "lFoot", reversal_planner_);

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
            &(((SwingPlanningCtrl*)right_swing_ctrl_)->curr_foot_pos_des_),
            VECT3, "rfoot_pos_des", 3);
    DataManager::GetDataManager()->RegisterData(
            &(((SwingPlanningCtrl*)left_swing_ctrl_)->curr_foot_pos_des_),
            VECT3, "lfoot_pos_des", 3);

    DataManager::GetDataManager()->RegisterData(
            &(((SwingPlanningCtrl*)right_swing_ctrl_)->curr_foot_vel_des_),
            VECT3, "rfoot_vel_des", 3);
    DataManager::GetDataManager()->RegisterData(
            &(((SwingPlanningCtrl*)left_swing_ctrl_)->curr_foot_vel_des_),
            VECT3, "lfoot_vel_des", 3);

    DataManager::GetDataManager()->RegisterData(
            &(((SwingPlanningCtrl*)right_swing_ctrl_)->curr_foot_acc_des_),
            VECT3, "rfoot_acc_des", 3);
    DataManager::GetDataManager()->RegisterData(
            &(((SwingPlanningCtrl*)left_swing_ctrl_)->curr_foot_acc_des_),
            VECT3, "lfoot_acc_des", 3);
}

RLWalkingTest::~RLWalkingTest(){
    for(int i(0); i<state_list_.size(); ++i){
        delete state_list_[i];
    }
}

void RLWalkingTest::TestInitialization(){
    reversal_planner_->PlannerInitialization(cfg_["planner_configuration"]["velocity_reversal_pln"]);

    jpos_ctrl_->ctrlInitialization(cfg_["control_configuration"]["joint_position_ctrl"]);
    body_up_ctrl_->ctrlInitialization(cfg_["control_configuration"]["double_contact_trans_ctrl"]);
    body_fix_ctrl_->ctrlInitialization(cfg_["control_configuration"]["body_ctrl"]);

    right_swing_start_trans_ctrl_->ctrlInitialization(cfg_["control_configuration"]["single_contact_trans_ctrl"]);
    right_swing_end_trans_ctrl_->ctrlInitialization(cfg_["control_configuration"]["single_contact_trans_ctrl"]);
    left_swing_start_trans_ctrl_->ctrlInitialization(cfg_["control_configuration"]["single_contact_trans_ctrl"]);
    left_swing_end_trans_ctrl_->ctrlInitialization(cfg_["control_configuration"]["single_contact_trans_ctrl"]);

    right_swing_ctrl_->ctrlInitialization(cfg_["control_configuration"]["right_body_foot_planning_ctrl"]);
    left_swing_ctrl_->ctrlInitialization(cfg_["control_configuration"]["left_body_foot_planning_ctrl"]);
}

int RLWalkingTest::_NextPhase(const int & phase){
    int next_phase = phase + 1;
    myUtils::color_print(myColor::BoldGreen, "[Phase " + std::to_string(next_phase) + "]");
    Eigen::Vector3d next_local_frame_location;

    if(phase == WkPhase::double_contact_1) {
        ++num_step_;
        printf("%i th step : ", num_step_);
        printf("Right Leg Swing\n");
        sp_->stance_foot = "lFoot";

        // Global Frame Update
        next_local_frame_location = robot_->getBodyNodeIsometry(DracoBodyNode::lFootCenter).translation();
        sp_->global_pos_local += next_local_frame_location;
        //myUtils::pretty_print(sp_->global_pos_local, std::cout, "****lstance");
    }
    if(phase == WkPhase::double_contact_2){
        ++num_step_;
        printf("%i th step : ", num_step_);
        printf("Left Leg Swing\n");

        sp_->stance_foot = "rFoot";

        // Global Frame Update
        next_local_frame_location = robot_->getBodyNodeIsometry(DracoBodyNode::rFootCenter).translation();
        sp_->global_pos_local += next_local_frame_location;
        //myUtils::pretty_print(sp_->global_pos_local, std::cout, "****rstance");
    }
    sp_->num_step_copy = num_step_;

    if(next_phase == WkPhase::NUM_WALKING_PHASE) {
        return WkPhase::double_contact_1;
    }
    else{
        return next_phase;
    }
}

void RLWalkingTest::_SettingParameter(){
    try {

        double tmp; bool b_tmp; Eigen::VectorXd tmp_vec(10); std::string tmp_str;

        YAML::Node test_cfg = cfg_["test_configuration"];

        myUtils::readParameter(test_cfg, "start_phase", phase_);

        myUtils::readParameter(test_cfg, "initial_jpos", tmp_vec);
        ((JPosTargetCtrl*)jpos_ctrl_)->setTargetPosition(tmp_vec);

        myUtils::readParameter(test_cfg, "body_height", tmp);
        ((DoubleContactTransCtrl*)body_up_ctrl_)->setStanceHeight(tmp);
        ((BodyCtrl*)body_fix_ctrl_)->setStanceHeight(tmp);
        ((SingleContactTransCtrl*)right_swing_start_trans_ctrl_)->setStanceHeight(tmp);
        ((SingleContactTransCtrl*)right_swing_end_trans_ctrl_)->setStanceHeight(tmp);
        ((SingleContactTransCtrl*)left_swing_start_trans_ctrl_)->setStanceHeight(tmp);
        ((SingleContactTransCtrl*)left_swing_end_trans_ctrl_)->setStanceHeight(tmp);
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
        ((SingleContactTransCtrl*)right_swing_start_trans_ctrl_)->setTransitionTime(tmp);
        ((SingleContactTransCtrl*)right_swing_end_trans_ctrl_)->setTransitionTime(tmp);
        ((SingleContactTransCtrl*)left_swing_start_trans_ctrl_)->setTransitionTime(tmp);
        ((SingleContactTransCtrl*)left_swing_end_trans_ctrl_)->setTransitionTime(tmp);
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

    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
        exit(0);
    }
}
