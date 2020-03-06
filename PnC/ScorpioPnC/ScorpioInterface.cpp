#include <math.h>
#include <stdio.h>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/ScorpioPnC/ScorpioInterface.hpp>
#include <PnC/ScorpioPnC/TestSet/OSCTest.hpp>
#include <PnC/ScorpioPnC/TestSet/GraspingTest.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>
#include <PnC/ScorpioPnC/ScorpioStateProvider.hpp>
#include <string>

ScorpioInterface::ScorpioInterface() : EnvInterface() {
    std::string border = "=";
    for (int i = 0; i < 79; ++i) {
        border += "=";
    }
    myUtils::color_print(myColor::BoldCyan, border);
    myUtils::pretty_constructor(0, "Scorpio Interface");

    robot_ = new RobotSystem(
        4, THIS_COM "RobotModel/Robot/Scorpio/Scorpio_Kin.urdf");
     //robot_->printRobotInfo();
    sp_ = ScorpioStateProvider::getStateProvider(robot_);

    count_ = 0;
    //waiting_count_ = 2;
    test_initialized = false;
    cmd_jpos_ = Eigen::VectorXd::Zero(Scorpio::n_adof);
    cmd_jvel_ = Eigen::VectorXd::Zero(Scorpio::n_adof);
    cmd_jtrq_ = Eigen::VectorXd::Zero(Scorpio::n_adof);

    _ParameterSetting();

    myUtils::color_print(myColor::BoldCyan, border);

    DataManager* data_manager = DataManager::GetDataManager();
    data_manager->RegisterData(&running_time_,DOUBLE,"time",1);
}

ScorpioInterface::~ScorpioInterface() {
    delete robot_;
    delete test_;
}

void ScorpioInterface::getCommand(void* _data, void* _command) { 
    ScorpioCommand* cmd = ((ScorpioCommand*)_command);
    ScorpioSensorData* data = ((ScorpioSensorData*)_data);

    if (!Initialization_(data, cmd)) {
        //state_estimator_->Update(data);
        robot_->updateSystem(data->q, data->qdot, true);
        test_->getCommand(cmd);
        //CropTorque_(cmd);
    }

    ++count_;
    running_time_ = (double)(count_)*ScorpioAux::ServoRate;
    sp_->curr_time = running_time_;
    sp_->phase_copy = test_->getPhase(); 
}

void ScorpioInterface::_ParameterSetting() {
    try {
        YAML::Node cfg =
            YAML::LoadFile(THIS_COM "Config/Scorpio/INTERFACE.yaml");
        std::string test_name =
            myUtils::readParameter<std::string>(cfg, "test_name");
        if (test_name == "osc_test") {
            test_ = new OSCTest(robot_);
        }
        else if (test_name == "grasping_test") {
            test_ = new GraspingTest(robot_);
        }
        else {
            printf(
                "[Scorpio Interface] There is no test matching test with "
                "the name\n");
            exit(0);
        }
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}

bool ScorpioInterface::Initialization_(ScorpioSensorData* _sensor_data,
                                        ScorpioCommand* _command) {
    if (!test_initialized) {
        test_->TestInitialization();
        test_initialized = true;
        DataManager::GetDataManager()->start();
    }
    //if (count_ < waiting_count_) {
        //state_estimator_->Initialization(_sensor_data);
        //DataManager::GetDataManager()->start();
        //return true;
    //}
    return false;
}

bool ScorpioInterface::IsReadyToMove(){
    if (!(sp_->is_closing) && !(sp_->is_opening) && !(sp_->is_moving)) {
        return true;
    } else{
        return false;
    }
}

void ScorpioInterface::MoveEndEffectorTo(double x, double y, double z) {
    if (sp_->phase_copy == GRASPING_TEST_PHASE::HOLD_PH && !(sp_->is_opening) && !(sp_->is_closing) || sp_->is_holding) {
        sp_->is_moving = true;
        Eigen::VectorXd des_pos = Eigen::VectorXd::Zero(3);
        des_pos << x, y, z;
        ((GraspingTest*)test_)->SetMovingTarget(des_pos);
    } else {
        std::cout << "Wait" << std::endl;
    }
}


bool ScorpioInterface::IsReadyToGrasp(){
    if (!(sp_->is_moving) && !(sp_->is_closing) && !(sp_->is_holding) && !(sp_->is_opening)) {
        return true;
    } else {
        return false;
    }
}

bool ScorpioInterface::IsReadyToRelease(){
    if (!(sp_->is_moving) && !(sp_->is_closing) && !(sp_->is_holding) && !(sp_->is_opening)) {
        return true;
    } else {
        return false;
    }
}

void ScorpioInterface::Grasp(){
    if (sp_->phase_copy == GRASPING_TEST_PHASE::HOLD_PH)  {
       sp_->is_closing = true;
       sp_->closing_opening_start_time = sp_->curr_time;
       //if()
    }else{
    std::cout << "Wait" << std::endl;
    }
}

void ScorpioInterface::Release(){
    if (sp_->phase_copy == GRASPING_TEST_PHASE::HOLD_PH && !(sp_->is_opening) && !(sp_->is_closing) && (sp_->is_holding)) {
       sp_->is_opening = true;
       sp_->is_holding = false;
       sp_->closing_opening_start_time = sp_->curr_time;
    }else{
        std::cout << "Wait" << std::endl;
    }
}
