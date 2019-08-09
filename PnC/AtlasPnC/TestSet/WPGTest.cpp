#include <PnC/AtlasPnC/AtlasDefinition.hpp>
#include <PnC/AtlasPnC/AtlasStateProvider.hpp>
#include <PnC/AtlasPnC/ContactSet/ContactSet.hpp>
#include <PnC/AtlasPnC/CtrlSet/CtrlSet.hpp>
#include <PnC/AtlasPnC/TaskSet/TaskSet.hpp>
#include <PnC/AtlasPnC/TestSet/WPGTest.hpp>
#include <PnC/PlannerSet/CentroidPlanner/CentroidPlanner.hpp>
#include <Utils/IO/DataManager.hpp>

WPGTest::WPGTest(RobotSystem* robot) : Test(robot) {
    myUtils::pretty_constructor(1, "WPG Test");
    cfg_ = YAML::LoadFile(THIS_COM "Config/Atlas/TEST/WPG_TEST.yaml");

    num_step_ = 0;
    sp_ = AtlasStateProvider::getStateProvider(robot_);
    sp_->stance_foot = AtlasBodyNode::l_sole;
    sp_->global_pos_local[0] = -0.02;
    sp_->global_pos_local[1] = 0.11;

    centroid_planner_param_ = new CentroidPlannerParameter(
        cfg_["planner_configuration"], robot_->getRobotMass());
    centroid_planner_ = new CentroidPlanner(centroid_planner_param_);

    phase_ = WPGPhase::double_contact_1;

    state_list_.clear();

    ds_ctrl_ = new DoubleSupportCtrl(robot, centroid_planner_);

    _SettingParameter();

    state_list_.push_back(ds_ctrl_);
}

WPGTest::~WPGTest() {
    delete centroid_planner_;
    delete centroid_planner_param_;
    delete ds_ctrl_;
}

void WPGTest::TestInitialization() {
    ds_ctrl_->ctrlInitialization(cfg_["control_configuration"]["ds_ctrl"]);
}

int WPGTest::_NextPhase(const int& phase) {
    int next_phase = phase + 1;
    myUtils::color_print(myColor::BoldGreen,
                         "[Phase " + std::to_string(next_phase) + "]");
    Eigen::Vector3d next_local_frame_location;

    if (phase == WPGPhase::double_contact_1) {
        ++num_step_;
        printf("%i th step : ", num_step_);
        printf("Right Leg Swing\n");
        sp_->stance_foot = AtlasBodyNode::l_sole;

        // Global Frame Update
        next_local_frame_location =
            robot_->getBodyNodeIsometry(AtlasBodyNode::l_sole).translation();
        sp_->global_pos_local += next_local_frame_location;
    }
    if (phase == WPGPhase::double_contact_2) {
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

    if (next_phase == WPGPhase::NUM_WALKING_PHASE) {
        return WPGPhase::double_contact_1;
    } else {
        return next_phase;
    }
}

void WPGTest::_SettingParameter() {
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

void WPGTest::AdditionalUpdate_() {}
