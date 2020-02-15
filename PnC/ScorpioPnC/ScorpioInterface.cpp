#include <math.h>
#include <stdio.h>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/ScorpioPnC/ScorpioInterface.hpp>
#include <PnC/ScorpioPnC/OSCTest.hpp>
//#include <PnC/DracoPnC/DracoStateEstimator.hpp>
//#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>
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
    //state_estimator_ = new DracoStateEstimator(robot_);
    //sp_ = DracoStateProvider::getStateProvider(robot_);

    //sp_->stance_foot = DracoBodyNode::lFootCenter;

    count_ = 0;
    //waiting_count_ = 2;
    test_initialized = false;
    cmd_jpos_ = Eigen::VectorXd::Zero(Scorpio::n_adof);
    cmd_jvel_ = Eigen::VectorXd::Zero(Scorpio::n_adof);
    cmd_jtrq_ = Eigen::VectorXd::Zero(Scorpio::n_adof);

    _ParameterSetting();

    myUtils::color_print(myColor::BoldCyan, border);

    //DataManager* data_manager = DataManager::GetDataManager();
    //data_manager->RegisterData(&cmd_jpos_, VECT, "jpos_des", Draco::n_adof);
    //data_manager->RegisterData(&cmd_jvel_, VECT, "jvel_des", Draco::n_adof);
    //data_manager->RegisterData(&cmd_jtrq_, VECT, "command", Draco::n_adof);
}

ScorpioInterface::~ScorpioInterface() {
    delete robot_;
    delete test_;
    //delete state_estimator_;
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

    //cmd_jtrq_ = cmd->jtrq;
    //cmd_jvel_ = cmd->qdot;
    //cmd_jpos_ = cmd->q;

    //std::cout << cmd_jtrq_ << std::endl;
    //std::cout << "========" << std::endl;
    //std::cout << cmd_jvel_ << std::endl;
    //std::cout << "========" << std::endl;
    //std::cout << cmd_jpos_ << std::endl;
    //std::cout << "========" << std::endl;

    ++count_;
    running_time_ = (double)(count_)*ScorpioAux::ServoRate;
    //sp_->curr_time = running_time_;
    //sp_->phase_copy = test_->getPhase();

}

//void ScorpioInterface::CropTorque_(DracoCommand* cmd) {
    // cmd->jtrq = myUtils::CropVector(cmd->jtrq,
    // robot_->GetTorqueLowerLimits(), robot_->GetTorqueUpperLimits(), "clip trq
    // in interface");
//}

void ScorpioInterface::_ParameterSetting() {
    try {
        YAML::Node cfg =
            YAML::LoadFile(THIS_COM "Config/Scorpio/INTERFACE.yaml");
        std::string test_name =
            myUtils::readParameter<std::string>(cfg, "test_name");
        if (test_name == "osc_test") {
            test_ = new OSCTest(robot_);
        } 
        //else if (test_name == "walking_test") {
            //test_ = new WalkingTest(robot_);
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
    }
    //if (count_ < waiting_count_) {
        //state_estimator_->Initialization(_sensor_data);
        //DataManager::GetDataManager()->start();
        //return true;
    //}
    return false;
}

