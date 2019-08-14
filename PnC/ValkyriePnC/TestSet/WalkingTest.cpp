#include <PnC/PlannerSet/CentroidPlanner/CentroidPlanner.hpp>
#include <PnC/PlannerSet/ContactSequenceGenerator/FootstepSequenceGenerator.hpp>
#include <PnC/ValkyriePnC/ContactSet/ContactSet.hpp>
#include <PnC/ValkyriePnC/CtrlSet/CtrlSet.hpp>
#include <PnC/ValkyriePnC/TaskSet/TaskSet.hpp>
#include <PnC/ValkyriePnC/TestSet/WalkingTest.hpp>
#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>
#include <Utils/IO/DataManager.hpp>

WalkingTest::WalkingTest(RobotSystem* robot) : Test(robot) {
    myUtils::pretty_constructor(1, "Walking Test");
    cfg_ = YAML::LoadFile(THIS_COM "Config/Valkyrie/TEST/WALKING_TEST.yaml");

    num_step_ = 0;
    sp_ = ValkyrieStateProvider::getStateProvider(robot_);
    sp_->stance_foot = ValkyrieBodyNode::leftCOP_Frame;

    centroid_planner_param_ = new CentroidPlannerParameter(
        cfg_["planner_configuration"], robot_->getRobotMass());
    centroid_planner_ = new CentroidPlanner(centroid_planner_param_);
    foot_sequence_gen_ = new FootstepSequenceGenerator();

    phase_ = WKPhase::double_contact_1;

    state_list_.clear();

    ds_ctrl_ =
        new DoubleSupportCtrl(robot, centroid_planner_, foot_sequence_gen_);

    r_ss_ctrl_ = new SingleSupportCtrl(robot, centroid_planner_,
                                       ValkyrieBodyNode::rightFoot);
    l_ss_ctrl_ = new SingleSupportCtrl(robot, centroid_planner_,
                                       ValkyrieBodyNode::leftFoot);

    rs_start_trns_ctrl_ = new TransitionCtrl(
        robot, centroid_planner_, ValkyrieBodyNode::rightFoot, false);
    rs_end_trns_ctrl_ = new TransitionCtrl(robot, centroid_planner_,
                                           ValkyrieBodyNode::rightFoot, true);
    ls_start_trns_ctrl_ = new TransitionCtrl(robot, centroid_planner_,
                                             ValkyrieBodyNode::leftFoot, false);
    ls_end_trns_ctrl_ = new TransitionCtrl(robot, centroid_planner_,
                                           ValkyrieBodyNode::leftFoot, true);

    _SettingParameter();

    state_list_.push_back(ds_ctrl_);
    state_list_.push_back(rs_start_trns_ctrl_);
    state_list_.push_back(r_ss_ctrl_);
    state_list_.push_back(rs_end_trns_ctrl_);
    state_list_.push_back(ds_ctrl_);
    state_list_.push_back(ls_start_trns_ctrl_);
    state_list_.push_back(l_ss_ctrl_);
    state_list_.push_back(ls_end_trns_ctrl_);
}

WalkingTest::~WalkingTest() {
    delete centroid_planner_;
    delete foot_sequence_gen_;
    delete centroid_planner_param_;

    delete ds_ctrl_;
    delete rs_start_trns_ctrl_;
    delete rs_end_trns_ctrl_;
    delete ls_start_trns_ctrl_;
    delete ls_end_trns_ctrl_;
    delete l_ss_ctrl_;
    delete r_ss_ctrl_;
}

void WalkingTest::TestInitialization() {
    ds_ctrl_->ctrlInitialization(cfg_["control_configuration"]["ds_ctrl"]);

    rs_start_trns_ctrl_->ctrlInitialization(
        cfg_["control_configuration"]["rs_start_trns_ctrl"]);
    rs_end_trns_ctrl_->ctrlInitialization(
        cfg_["control_configuration"]["rs_end_trns_ctrl"]);
    ls_start_trns_ctrl_->ctrlInitialization(
        cfg_["control_configuration"]["ls_start_trns_ctrl"]);
    ls_end_trns_ctrl_->ctrlInitialization(
        cfg_["control_configuration"]["ls_end_trns_ctrl"]);

    r_ss_ctrl_->ctrlInitialization(cfg_["control_configuration"]["r_ss_ctrl"]);
    l_ss_ctrl_->ctrlInitialization(cfg_["control_configuration"]["l_ss_ctrl"]);
}

int WalkingTest::_NextPhase(const int& phase) {
    int next_phase = phase + 1;
    myUtils::color_print(myColor::BoldGreen,
                         "[Phase " + std::to_string(next_phase) + "]");
    Eigen::Vector3d next_local_frame_location;

    if (phase == WKPhase::double_contact_1) {
        ++num_step_;
        printf("%i th step : ", num_step_);
        printf("Right Leg Swing\n");
        sp_->stance_foot = ValkyrieBodyNode::leftCOP_Frame;

        // Global Frame Update
        next_local_frame_location =
            robot_->getBodyNodeIsometry(ValkyrieBodyNode::leftCOP_Frame)
                .translation();
    }
    if (phase == WKPhase::double_contact_2) {
        ++num_step_;
        printf("%i th step : ", num_step_);
        printf("Left Leg Swing\n");

        sp_->stance_foot = ValkyrieBodyNode::rightCOP_Frame;

        // Global Frame Update
        next_local_frame_location =
            robot_->getBodyNodeIsometry(ValkyrieBodyNode::rightCOP_Frame)
                .translation();
    }
    sp_->num_step_copy = num_step_;

    if (next_phase == WKPhase::NUM_WALKING_PHASE) {
        return WKPhase::double_contact_1;
    } else {
        return next_phase;
    }
}

void WalkingTest::_SettingParameter() {
    try {
        double tmp;
        bool b_tmp;
        Eigen::VectorXd tmp_vec;
        std::string tmp_str;

        YAML::Node test_cfg = cfg_["test_configuration"];
        myUtils::readParameter(test_cfg, "dsp_duration", tmp);
        ((DoubleSupportCtrl*)ds_ctrl_)->SetDSPDuration(tmp);
        ((SingleSupportCtrl*)r_ss_ctrl_)->SetDSPDuration(tmp);
        ((SingleSupportCtrl*)l_ss_ctrl_)->SetDSPDuration(tmp);
        ((TransitionCtrl*)rs_start_trns_ctrl_)->SetDSPDuration(tmp);
        ((TransitionCtrl*)ls_start_trns_ctrl_)->SetDSPDuration(tmp);
        ((TransitionCtrl*)rs_end_trns_ctrl_)->SetDSPDuration(tmp);
        ((TransitionCtrl*)ls_end_trns_ctrl_)->SetDSPDuration(tmp);

        myUtils::readParameter(test_cfg, "ssp_duration", tmp);
        ((DoubleSupportCtrl*)ds_ctrl_)->SetSSPDuration(tmp);
        ((SingleSupportCtrl*)r_ss_ctrl_)->SetSSPDuration(tmp);
        ((SingleSupportCtrl*)l_ss_ctrl_)->SetSSPDuration(tmp);
        ((TransitionCtrl*)rs_start_trns_ctrl_)->SetSSPDuration(tmp);
        ((TransitionCtrl*)ls_start_trns_ctrl_)->SetSSPDuration(tmp);
        ((TransitionCtrl*)rs_end_trns_ctrl_)->SetSSPDuration(tmp);
        ((TransitionCtrl*)ls_end_trns_ctrl_)->SetSSPDuration(tmp);

        myUtils::readParameter(test_cfg, "ini_dsp_duration", tmp);
        ((DoubleSupportCtrl*)ds_ctrl_)->SetIniDSPDuration(tmp);
        ((TransitionCtrl*)rs_start_trns_ctrl_)->SetIniDSPDuration(tmp);
        ((TransitionCtrl*)ls_start_trns_ctrl_)->SetIniDSPDuration(tmp);
        ((TransitionCtrl*)rs_end_trns_ctrl_)->SetIniDSPDuration(tmp);
        ((TransitionCtrl*)ls_end_trns_ctrl_)->SetIniDSPDuration(tmp);

        myUtils::readParameter(test_cfg, "fin_dsp_duration", tmp);
        ((DoubleSupportCtrl*)ds_ctrl_)->SetFinDSPDuration(tmp);

        myUtils::readParameter(test_cfg, "footstep_length", tmp);
        ((DoubleSupportCtrl*)ds_ctrl_)->SetFootStepLength(tmp);
        ((SingleSupportCtrl*)r_ss_ctrl_)->SetFootStepLength(tmp);
        ((SingleSupportCtrl*)l_ss_ctrl_)->SetFootStepLength(tmp);
        foot_sequence_gen_->SetFootStepLength(tmp);

        myUtils::readParameter(test_cfg, "footstep_width", tmp);
        ((DoubleSupportCtrl*)ds_ctrl_)->SetFootStepWidth(tmp);
        ((SingleSupportCtrl*)r_ss_ctrl_)->SetFootStepWidth(tmp);
        ((SingleSupportCtrl*)l_ss_ctrl_)->SetFootStepWidth(tmp);
        foot_sequence_gen_->SetFootStepWidth(tmp);

        myUtils::readParameter(test_cfg, "com_height", tmp);
        ((DoubleSupportCtrl*)ds_ctrl_)->SetCoMHeight(tmp);
        ((SingleSupportCtrl*)r_ss_ctrl_)->SetCoMHeight(tmp);
        ((SingleSupportCtrl*)l_ss_ctrl_)->SetCoMHeight(tmp);
        ((TransitionCtrl*)rs_start_trns_ctrl_)->SetCoMHeight(tmp);
        ((TransitionCtrl*)ls_start_trns_ctrl_)->SetCoMHeight(tmp);
        ((TransitionCtrl*)rs_end_trns_ctrl_)->SetCoMHeight(tmp);
        ((TransitionCtrl*)ls_end_trns_ctrl_)->SetCoMHeight(tmp);

        myUtils::readParameter(test_cfg, "walking_distance", tmp);
        ((DoubleSupportCtrl*)ds_ctrl_)->SetWalkingDistance(tmp);

        myUtils::readParameter(test_cfg, "trns_duration", tmp);
        ((TransitionCtrl*)rs_start_trns_ctrl_)->SetTRNSDuration(tmp);
        ((TransitionCtrl*)ls_start_trns_ctrl_)->SetTRNSDuration(tmp);
        ((TransitionCtrl*)rs_end_trns_ctrl_)->SetTRNSDuration(tmp);
        ((TransitionCtrl*)ls_end_trns_ctrl_)->SetTRNSDuration(tmp);

        myUtils::readParameter(test_cfg, "replan", b_tmp);
        ((DoubleSupportCtrl*)ds_ctrl_)->SetRePlanningFlag(b_tmp);
        ((SingleSupportCtrl*)r_ss_ctrl_)->SetRePlanningFlag(b_tmp);
        ((SingleSupportCtrl*)l_ss_ctrl_)->SetRePlanningFlag(b_tmp);

        myUtils::readParameter(test_cfg, "swing_height", tmp);
        ((SingleSupportCtrl*)r_ss_ctrl_)->SetSwingHeight(tmp);
        ((SingleSupportCtrl*)l_ss_ctrl_)->SetSwingHeight(tmp);

    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}

void WalkingTest::AdditionalUpdate_() {}

