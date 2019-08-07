#include <PnC/PlannerSet/CentroidPlanner/CentroidPlanner.hpp>
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

    phase_ = WKPhase::double_contact_1;

    state_list_.clear();

    ds_ctrl_ = new DoubleSupportCtrl(robot, centroid_planner_);

    _SettingParameter();

    state_list_.push_back(ds_ctrl_);
}

WalkingTest::~WalkingTest() {
    delete centroid_planner_;
    delete centroid_planner_param_;
    delete ds_ctrl_;
}

void WalkingTest::TestInitialization() {
    ds_ctrl_->ctrlInitialization(cfg_["control_configuration"]["ds_ctrl"]);
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

        myUtils::readParameter(test_cfg, "ssp_duration", tmp);
        ((DoubleSupportCtrl*)ds_ctrl_)->SetSSPDuration(tmp);

        myUtils::readParameter(test_cfg, "ini_dsp_duration", tmp);
        ((DoubleSupportCtrl*)ds_ctrl_)->SetIniDSPDuration(tmp);

        myUtils::readParameter(test_cfg, "footstep_length", tmp_vec);
        ((DoubleSupportCtrl*)ds_ctrl_)->SetFootStepLength(tmp_vec);

        myUtils::readParameter(test_cfg, "com_height", tmp);
        ((DoubleSupportCtrl*)ds_ctrl_)->SetCoMHeight(tmp);
        //((DoubleSupportCtrl*)ssp_ctrl_)->SetDuration(tmp);
        // myUtils::readParameter(test_cfg, "tr_duration", tmp);
        //((DoubleSupportCtrl*)tr_ctrl_)->SetDuration(tmp);

    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}

void WalkingTest::AdditionalUpdate_() {}

