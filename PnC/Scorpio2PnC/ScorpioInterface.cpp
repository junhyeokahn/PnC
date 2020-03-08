#include <math.h>
#include <stdio.h>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/Scorpio2PnC/ScorpioInterface.hpp>
#include <PnC/Scorpio2PnC/TestSet/OSCTest.hpp>
#include <PnC/Scorpio2PnC/TestSet/GraspingTest.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>
#include <PnC/Scorpio2PnC/ScorpioStateProvider.hpp>
#include <string>

Scorpio2Interface::Scorpio2Interface() : EnvInterface() {
    std::string border = "=";
    for (int i = 0; i < 79; ++i) {
        border += "=";
    }
    myUtils::color_print(myColor::BoldCyan, border);
    myUtils::pretty_constructor(0, "Scorpio2 Interface");

    robot_ = new RobotSystem(
        4, THIS_COM "RobotModel/Robot/Scorpio/Scorpio_Kin.urdf");
     //robot_->printRobotInfo();
    sp_ = Scorpio2StateProvider::getStateProvider(robot_);

    count_ = 0;
    //waiting_count_ = 2;
    test_initialized = false;
    cmd_jpos_ = Eigen::VectorXd::Zero(Scorpio2::n_adof);
    cmd_jvel_ = Eigen::VectorXd::Zero(Scorpio2::n_adof);
    cmd_jtrq_ = Eigen::VectorXd::Zero(Scorpio2::n_adof);

    _ParameterSetting();

    myUtils::color_print(myColor::BoldCyan, border);

    DataManager* data_manager = DataManager::GetDataManager();
    data_manager->RegisterData(&running_time_,DOUBLE,"time",1);
}

Scorpio2Interface::~Scorpio2Interface() {
    delete robot_;
    delete test_;
}

void Scorpio2Interface::getCommand(void* _data, void* _command) { 
    Scorpio2Command* cmd = ((Scorpio2Command*)_command);
    Scorpio2SensorData* data = ((Scorpio2SensorData*)_data);

    if (!Initialization_(data, cmd)) {
        //state_estimator_->Update(data);
        robot_->updateSystem(data->q, data->qdot, true);
        test_->getCommand(cmd);
        //CropTorque_(cmd);
    }

    ++count_;
    running_time_ = (double)(count_)*Scorpio2Aux::ServoRate;
    sp_->curr_time = running_time_;
    sp_->phase_copy = test_->getPhase(); 
}

void Scorpio2Interface::_ParameterSetting() {
    try {
        YAML::Node cfg =
            YAML::LoadFile(THIS_COM "Config/Scorpio/INTERFACE.yaml");
        std::string test_name =
            myUtils::readParameter<std::string>(cfg, "test_name");
        if (test_name == "osc_test") {
            test_ = new OSCTest(robot_);
        }
        else if (test_name == "grasping_test") {
            test_ = new Grasping2Test(robot_);
        }
        else {
            printf(
                "[Scorpio2 Interface] There is no test matching test with "
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

bool Scorpio2Interface::Initialization_(Scorpio2SensorData* _sensor_data,
                                        Scorpio2Command* _command) {
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

bool Scorpio2Interface::IsReadyToMove(){
    if (!(sp_->is_closing) && !(sp_->is_opening) && !(sp_->is_moving)) {
        return true;
    } else{
        return false;
    }
}

void Scorpio2Interface::MoveEndEffectorTo(double x, double y, double z) {
    std::cout << "-------------------------------" << std::endl;
    std::cout << "Scorpio2 2 Move to : " << x << ", " << y << ", " << z << std::endl;
    if (sp_->phase_copy == GRASPING2_TEST_PHASE::HOLD2_PH && !(sp_->is_opening) && !(sp_->is_closing) || sp_->is_holding) {
        sp_->is_moving = true;
        Eigen::VectorXd des_pos = Eigen::VectorXd::Zero(3);
        des_pos << x, y, z;
        ((Grasping2Test*)test_)->SetMovingTarget(des_pos);
    } else {
        std::cout << "Wait" << std::endl;
    }
}


bool Scorpio2Interface::IsReadyToGrasp(){
    if (!(sp_->is_moving) && !(sp_->is_closing) && !(sp_->is_holding) && !(sp_->is_opening)) {
        return true;
    } else {
        return false;
    }
}

bool Scorpio2Interface::IsReadyToRelease(){
    if (!(sp_->is_moving) && !(sp_->is_closing) && !(sp_->is_holding) && !(sp_->is_opening)) {
        return true;
    } else {
        return false;
    }
}

void Scorpio2Interface::Grasp(){
    if (sp_->phase_copy == GRASPING2_TEST_PHASE::HOLD2_PH)  {
       sp_->is_closing = true;
       sp_->closing_opening_start_time = sp_->curr_time;
       //if()
    }else{
    std::cout << "Wait" << std::endl;
    }
}

void Scorpio2Interface::Release(){
    if (sp_->phase_copy == GRASPING2_TEST_PHASE::HOLD2_PH && !(sp_->is_opening) && !(sp_->is_closing) && (sp_->is_holding)) {
       sp_->is_opening = true;
       sp_->is_holding = false;
       sp_->closing_opening_start_time = sp_->curr_time;
    }else{
        std::cout << "Wait" << std::endl;
    }
}

void Scorpio2Interface::PrintPhase(){
    std::cout << "Scorpio2 Phase is: " << sp_->phase_copy << std::endl;
}
