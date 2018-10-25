#include <PnC/DracoPnC/TestSet/TestSet.hpp>
#include <PnC/DracoPnC/TaskSet/TaskSet.hpp>
#include <PnC/DracoPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/DracoPnC/ContactSet/ContactSet.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/PlannerSet/PIPM_FootPlacementPlanner/Reversal_LIPM_Planner.hpp>
#include <Utils/DataManager.hpp>

WalkingTest::WalkingTest(RobotSystem* robot) : Test(robot) {
    num_step_ = 0;
    sp_ = DracoStateProvider::getStateProvider(robot_);
    sp_->stance_foot = "lAnkle";
    sp_->global_pos_local[1] = 0.15;
    reversal_planner_ = new Reversal_LIPM_Planner();
    phase_ = WkPhase::initiation;

    state_list_.clear();

    jpos_ctrl_ = new JPosTargetCtrl(robot);
    body_up_ctrl_ = new DoubleContactTransCtrl(robot);
    body_fix_ctrl_ = new BodyCtrl(robot);
    // Swing Controller Selection
    right_swing_ctrl_ =
        new BodyFootPlanningCtrl(robot_, "rAnkle", reversal_planner_);
    left_swing_ctrl_ =
        new BodyFootPlanningCtrl(robot_, "lAnkle", reversal_planner_);

    // Right
    right_swing_start_trans_ctrl_ =
        new SingleContactTransCtrl(robot, "rAnkle", false);
    right_swing_end_trans_ctrl_ =
        new SingleContactTransCtrl(robot, "rAnkle", true);
    // Left
    left_swing_start_trans_ctrl_ =
        new SingleContactTransCtrl(robot, "lAnkle", false);
    left_swing_end_trans_ctrl_ =
        new SingleContactTransCtrl(robot, "lAnkle", true);


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

    printf("[Walking Config Test] Constructed\n");
}

WalkingTest::~WalkingTest(){
    for(int i(0); i<state_list_.size(); ++i){
        delete state_list_[i];
    }
}

void WalkingTest::TestInitialization(){
    reversal_planner_->PlannerInitialization("VELOCITY_REVERSAL_PLANNER");
    // Yaml file name
    jpos_ctrl_->ctrlInitialization("JOINT_CTRL");
    body_up_ctrl_->ctrlInitialization("DOUBLE_CONTACT_TRANS_CTRL");
    body_fix_ctrl_->ctrlInitialization("BODY_CTRL");
    // Transition
    right_swing_start_trans_ctrl_->ctrlInitialization("SINGLE_CONTACT_TRANS_CTRL");
    right_swing_end_trans_ctrl_->ctrlInitialization("SINGLE_CONTACT_TRANS_CTRL");
    left_swing_start_trans_ctrl_->ctrlInitialization("SINGLE_CONTACT_TRANS_CTRL");
    left_swing_end_trans_ctrl_->ctrlInitialization("SINGLE_CONTACT_TRANS_CTRL");
    // Swing
    right_swing_ctrl_->ctrlInitialization("RIGHT_BODY_FOOT_PLANNING_CTRL");
    left_swing_ctrl_->ctrlInitialization("LEFT_BODY_FOOT_PLANNING_CTRL");
}

int WalkingTest::_NextPhase(const int & phase){
    int next_phase = phase + 1;
    printf("next phase: %i\n", next_phase);
    Eigen::Vector3d next_local_frame_location;

    if(phase == WkPhase::double_contact_1) {
        ++num_step_;
        printf("%i th step:\n", num_step_);
        // printf("One swing done: Next Right Leg Swing\n");
        sp_->stance_foot = "lAnkle";

        // Global Frame Update
        next_local_frame_location = robot_->getBodyNodeIsometry("lAnkle").translation();
        sp_->global_pos_local += next_local_frame_location;
    }
    if(phase == WkPhase::double_contact_2){
        ++num_step_;
        printf("%i th step:\n", num_step_);

        sp_->stance_foot = "rAnkle";

        // Global Frame Update
        next_local_frame_location = robot_->getBodyNodeIsometry("rAnkle").translation();
        sp_->global_pos_local += next_local_frame_location;
    }
    sp_->num_step_copy = num_step_;

    if(next_phase == WkPhase::NUM_WALKING_PHASE) {
        return WkPhase::double_contact_1;
    }
    else{
        return next_phase;
    }
}

void WalkingTest::_SettingParameter(){
    try {

        double tmp; bool b_tmp; Eigen::VectorXd tmp_vec; std::string tmp_str;

        YAML::Node cfg = YAML::LoadFile(THIS_COM"Config/Draco/TEST/WALKING_TEST.yaml");

        myUtils::readParameter(cfg, "start_phase", phase_);

        myUtils::readParameter(cfg, "initial_jpos", tmp_vec);
        ((JPosTargetCtrl*)jpos_ctrl_)->setTargetPosition(tmp_vec);

        myUtils::readParameter(cfg, "body_height", tmp);
        ((DoubleContactTransCtrl*)body_up_ctrl_)->setStanceHeight(tmp);
        ((BodyCtrl*)body_fix_ctrl_)->setStanceHeight(tmp);
        ((SingleContactTransCtrl*)right_swing_start_trans_ctrl_)->setStanceHeight(tmp);
        ((SingleContactTransCtrl*)right_swing_end_trans_ctrl_)->setStanceHeight(tmp);
        ((SingleContactTransCtrl*)left_swing_start_trans_ctrl_)->setStanceHeight(tmp);
        ((SingleContactTransCtrl*)left_swing_end_trans_ctrl_)->setStanceHeight(tmp);
        ((SwingPlanningCtrl*)right_swing_ctrl_)->setStanceHeight(tmp);
        ((SwingPlanningCtrl*)left_swing_ctrl_)->setStanceHeight(tmp);
        ((Reversal_LIPM_Planner*)reversal_planner_)->setOmega(tmp);

        myUtils::readParameter(cfg, "jpos_initialization_time", tmp);
        ((JPosTargetCtrl*)jpos_ctrl_)->setMovingTime(tmp);

        myUtils::readParameter(cfg, "com_lifting_time", tmp);
        ((DoubleContactTransCtrl*)body_up_ctrl_)->setStanceTime(tmp);

        myUtils::readParameter(cfg, "stance_time", tmp);
        ((BodyCtrl*)body_fix_ctrl_)->setStanceTime(tmp);
        ((SwingPlanningCtrl*)right_swing_ctrl_)->notifyStanceTime(tmp);
        ((SwingPlanningCtrl*)left_swing_ctrl_)->notifyStanceTime(tmp);

        myUtils::readParameter(cfg, "swing_time", tmp);
        ((SwingPlanningCtrl*)right_swing_ctrl_)->setSwingTime(tmp);
        ((SwingPlanningCtrl*)left_swing_ctrl_)->setSwingTime(tmp);

        myUtils::readParameter(cfg, "st_transition_time", tmp);
        ((SingleContactTransCtrl*)right_swing_start_trans_ctrl_)->setTransitionTime(tmp);
        ((SingleContactTransCtrl*)right_swing_end_trans_ctrl_)->setTransitionTime(tmp);
        ((SingleContactTransCtrl*)left_swing_start_trans_ctrl_)->setTransitionTime(tmp);
        ((SingleContactTransCtrl*)left_swing_end_trans_ctrl_)->setTransitionTime(tmp);

        myUtils::readParameter(cfg, "replanning", b_tmp);
        ((SwingPlanningCtrl*)right_swing_ctrl_)->setReplanning(b_tmp);
        ((SwingPlanningCtrl*)left_swing_ctrl_)->setReplanning(b_tmp);

        myUtils::readParameter(cfg, "double_stance_mix_ratio", tmp);
        ((SwingPlanningCtrl*)right_swing_ctrl_)->setDoubleStanceRatio(tmp);
        ((SwingPlanningCtrl*)left_swing_ctrl_)->setDoubleStanceRatio(tmp);

        myUtils::readParameter(cfg, "transition_phase_mix_ratio", tmp);
        ((SwingPlanningCtrl*)right_swing_ctrl_)->setTransitionPhaseRatio(tmp);
        ((SwingPlanningCtrl*)left_swing_ctrl_)->setTransitionPhaseRatio(tmp);

        myUtils::readParameter(cfg, "contact_switch_check", b_tmp);
        ((SwingPlanningCtrl*)right_swing_ctrl_)->setContactSwitchCheck(b_tmp);
        ((SwingPlanningCtrl*)left_swing_ctrl_)->setContactSwitchCheck(b_tmp);

    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
        exit(0);
    }
}
